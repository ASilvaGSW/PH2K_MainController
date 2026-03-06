import candle_driver
import time
import threading
import queue

class Canbus:

    def __init__(self, device_index: int = 0, channel_index: int = 0, bitrate: int = 125000):
        self.device_index = device_index
        self.channel_index = channel_index
        self.bitrate = bitrate
        self.device = None
        self.channel = None
        self.is_started = False
        
        # Nuevos atributos para el lector en segundo plano
        self.incoming_messages_queue = queue.Queue()
        self._reader_thread = None
        self._stop_reader_event = threading.Event()
        self._send_lock = threading.Lock()  # Evita envíos concurrentes desde múltiples hilos

    def device_list(self):
        try:
            devices = candle_driver.list_devices()
            return devices
        except Exception as e:
            print(f"Error listing devices: {e}")
            return None

    def start_canbus(self):
        try:
            devices = candle_driver.list_devices()
            if not devices:
                print("No CAN devices found.")
                self.is_started = False
                return False
            self.device = devices[0]
            self.device.open()
            self.channel = self.device.channel(0)
            self.channel.set_bitrate(self.bitrate)
            self.channel.start()
            
            # Iniciar el hilo lector
            self._stop_reader_event.clear()
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            
            self.is_started = True
            return True
        except Exception as e:
            print(f"Error Starting Canbus: {e}")
            self.is_started = False
            return False

    def _reader_loop(self):
        """Bucle que se ejecuta en un hilo para leer mensajes CAN continuamente."""
        print("CAN reader thread started.")
        while not self._stop_reader_event.is_set():
            try:
                # Usamos un timeout pequeño para no bloquear indefinidamente y poder detener el hilo
                frame = self.channel.read(100)
                if frame[0] is not None:
                    self.incoming_messages_queue.put(frame)
            except Exception as e:
                # Si el canal se cierra o hay un error, detenemos el bucle
                if self.is_started:
                    print(f"Error in CAN reader thread: {e}")
                break
        print("CAN reader thread stopped.")

    def flush_buffer(self):
        """Limpia la cola de mensajes pendientes. NO se llama desde send_message.
        Cuidado en multi-hilo: puede eliminar respuestas que otros hilos esperan."""
        while not self.incoming_messages_queue.empty():
            try:
                self.incoming_messages_queue.get_nowait()
            except queue.Empty:
                break
        print("Message queue flushed.")

    def send_message(self, can_id, data, wait_for_reply=False, max_retries=None):
        """
        Send a CAN message and wait for a reply from the same CAN ID.
        Thread-safe: usa lock para envío y busca respuesta en la queue (no en el bus).
        Returns a tuple (status, reply_data) where:
            status: 'success', 'error', or 'not_found'
            reply_data: CAN data bytes if successful, None otherwise
        """
        try:
            # NO flush_buffer: otros hilos pueden estar esperando sus respuestas en la queue
            with self._send_lock:
                result = self.channel.write(can_id, bytes(data))
            
            if result is None:
                print(f"Failed to send message to CAN ID {can_id}")
                return 'error', None
            
            if not wait_for_reply:
                return 'success', None
            
            # Espera la respuesta usando el nuevo read_message con un timeout de 7 segundos
            reply_status, reply_data = self.read_message(7, can_id + 0x400, data[0])
            
            if reply_status == 'success':
                return 'success', reply_data
            elif reply_status == 'error':
                print(f"Error response received from CAN ID {can_id}")
                return 'error', reply_data
            else:  # 'not_found'
                print(f"No reply received from CAN ID {can_id}")
                return 'not_found', None
                
        except Exception as e:
            print(f"Error in send_message: {e}")
            return 'error', None

    def read_message(self, timeout_seconds, search_can_id, function_id=0x00):
        """
        Busca un mensaje específico en la cola (incoming_messages_queue), NO en el bus.
        El hilo lector es el único que lee del bus; las respuestas se consumen aquí.
        Thread-safe: queue.Queue es seguro para múltiples consumidores.
        Returns a tuple (status, reply_data).
        """
        start_time = time.time()
        
        # Lista temporal para mensajes no deseados
        unmatched_messages = []

        try:
            while time.time() - start_time < timeout_seconds:
                try:
                    # Espera un mensaje de la cola, con un timeout para no bloquear indefinidamente
                    frame = self.incoming_messages_queue.get(timeout=0.1)
                    frame_type, can_id, can_data, extended, ts = frame

                    if can_id == search_can_id and len(can_data) > 1 and function_id == can_data[0]:
                        # Devolver los mensajes no coincidentes a la cola
                        for msg in unmatched_messages:
                            self.incoming_messages_queue.put(msg)
                        
                        if can_data[1] == 1:
                            print(f"Valid response (success) from CAN ID {search_can_id}: {can_data}")
                            return 'success', can_data
                        elif can_data[1] in [2, 3, 4]:
                            print(f"Valid response (error) from CAN ID {search_can_id}: {can_data}")
                            return 'error', can_data
                    else:
                        # Si no es el mensaje que buscamos, lo guardamos temporalmente
                        unmatched_messages.append(frame)

                except queue.Empty:
                    # Si la cola está vacía, simplemente continuamos esperando
                    continue
            
            # Si se agota el tiempo, devolvemos los mensajes no coincidentes a la cola
            for msg in unmatched_messages:
                self.incoming_messages_queue.put(msg)
            
            print(f"No valid response from CAN ID {search_can_id} within {timeout_seconds}s.")
            return 'not_found', None

        except Exception as e:
            print(f"Error in read_message: {e}")
            # Devolver los mensajes no coincidentes a la cola en caso de error
            for msg in unmatched_messages:
                self.incoming_messages_queue.put(msg)
            return 'not_found', None

    def close_canbus(self):
        """Detiene el canal CAN y el hilo lector."""
        if self.is_started:
            self._stop_reader_event.set()  # Señal para detener el hilo
            if self._reader_thread and self._reader_thread.is_alive():
                self._reader_thread.join(timeout=1)  # Espera a que el hilo termine

            if self.channel:
                self.channel.stop()
            if self.device:
                self.device.close()
            
            self.is_started = False
            print("CAN bus and reader thread stopped.")
        

        