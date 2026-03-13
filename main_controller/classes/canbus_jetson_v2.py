import can
import time
import subprocess
import os
import threading
import queue

# Configurar la interfaz can0
# sudo modprobe can
# sudo modprobe can-raw
# sudo ip link set can0 up type can bitrate 125000

class Canbus:
    """
    Clase para manejar comunicación CAN en Jetson Orin Nano usando python-can con socketcan.
    Esta clase replica la interfaz de la implementación original basada en candle_driver.
    """

    def __init__(self, device_index: int = 0, channel_index: int = 0, bitrate: int = 125000):
        """
        Inicializa la clase Canbus.
        
        Args:
            device_index: No se utiliza en esta implementación, mantenido por compatibilidad
            channel_index: No se utiliza en esta implementación, mantenido por compatibilidad
            bitrate: Velocidad de comunicación en bits por segundo
        """
        self.device_index = device_index  # Mantenido por compatibilidad
        self.channel_index = channel_index  # Mantenido por compatibilidad
        self.bitrate = bitrate
        self.device = None  # Mantenido por compatibilidad
        self.channel = None  # Referencia al bus CAN
        self.is_started = False
        self.interface = 'can0'  # Interfaz socketcan por defecto en Jetson
        
        # Nuevos atributos para el lector en segundo plano
        # Buffer compartido de mensajes y candado para acceso concurrente
        self._messages = []  # lista de tuplas (msg, timestamp)
        self._messages_lock = threading.Lock()
        self._reader_thread = None
        self._stop_reader_event = threading.Event()
        self._send_lock = threading.Lock()  # Evita envíos concurrentes desde múltiples hilos
        self._cleaner_thread = None
        self._stop_cleaner_event = threading.Event()
        self._message_ttl = 30.0  # segundos que se conserva un mensaje en la cola
        
        # Configurar automáticamente la interfaz CAN al inicializar
        self.setup_can_interface()

    def setup_can_interface(self):
        """
        Configura automáticamente la interfaz CAN en el Jetson Orin Nano.
        Ejecuta los comandos necesarios para habilitar el bus CAN.
        
        Returns:
            True si la configuración fue exitosa, False en caso contrario
        """
        try:
            print("Configurando interfaz CAN en Jetson Orin Nano...")
            
            # Lista de comandos para configurar CAN con contraseña
            commands = [
                "echo 'root' | sudo -S modprobe can",
                "echo 'root' | sudo -S modprobe can-raw",
                "echo 'root' | sudo -S modprobe mttcan",
                f"echo 'root' | sudo -S ip link set {self.interface} down",
                f"echo 'root' | sudo -S ip link set {self.interface} type can bitrate {self.bitrate}",
                f"echo 'root' | sudo -S ip link set {self.interface} up"
            ]
            
            for cmd in commands:
                try:
                    # print(f"Ejecutando: {cmd}")
                    result = subprocess.run(
                        cmd,
                        shell=True,
                        capture_output=True,
                        text=True,
                        timeout=10
                    )
                    
                    if result.returncode != 0:
                        print(f"Advertencia en comando '{cmd}': {result.stderr}")
                        # Algunos comandos pueden fallar si ya están configurados, continuamos
                        
                except subprocess.TimeoutExpired:
                    print(f"Timeout en comando: {cmd}")
                except Exception as e:
                    print(f"Error ejecutando '{cmd}': {e}")
            
            # Verificar si la interfaz está disponible
            try:
                result = subprocess.run(
                    ["ip", "link", "show", self.interface],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0:
                    print(f"Interfaz {self.interface} configurada correctamente")
                    return True
                else:
                    print(f"Error: Interfaz {self.interface} no disponible")
                    return False
                    
            except Exception as e:
                print(f"Error verificando interfaz: {e}")
                return False
                
        except Exception as e:
            print(f"Error en setup_can_interface: {e}")
            return False

    def device_list(self):
        """
        Lista los dispositivos CAN disponibles.
        En esta implementación, devuelve una lista con la interfaz socketcan configurada.
        
        Returns:
            Lista con la interfaz configurada o None en caso de error
        """
        try:
            # En socketcan, no hay una forma directa de listar dispositivos como en candle_driver
            # Devolvemos una lista con la interfaz configurada
            return [self.interface]
        except Exception as e:
            print(f"Error listing devices: {e}")
            return None

    def start_canbus(self):
        """
        Inicia la comunicación CAN.
        
        Returns:
            True si se inició correctamente, False en caso contrario
        """
        try:
            # Crear una instancia de Bus con la interfaz socketcan
            self.channel = can.interface.Bus(
                bustype='socketcan',
                channel=self.interface,
                bitrate=self.bitrate
            )
            
            # Iniciar el hilo lector
            self._stop_reader_event.clear()
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            
            # Iniciar el hilo de limpieza de mensajes expirados
            self._stop_cleaner_event.clear()
            self._cleaner_thread = threading.Thread(target=self._cleaner_loop, daemon=True)
            self._cleaner_thread.start()
            
            self.is_started = True
            return True
        except can.CanError as e:
            print(f"Error Starting Canbus: {e}")
            self.is_started = False
            return False
        except Exception as e:
            print(f"Unexpected error Starting Canbus: {e}")
            self.is_started = False
            return False

    def _reader_loop(self):
        """Bucle que se ejecuta en un hilo para leer mensajes CAN continuamente."""
        print("CAN reader thread started.")
        while not self._stop_reader_event.is_set():
            try:
                msg = self.channel.recv(0.05)  # Timeout de 1 segundo para no bloquear
                if msg is not None:
                    # Guardamos el mensaje junto con un timestamp en el buffer compartido
                    with self._messages_lock:
                        self._messages.append((msg, time.time()))
            except Exception as e:
                if self.is_started:
                    print(f"Error in CAN reader thread: {e}")
                break
        print("CAN reader thread stopped.")

    def flush_buffer(self):
        """Limpia el buffer de mensajes pendientes."""
        with self._messages_lock:
            self._messages.clear()
        print("Message buffer flushed.")

    def _purge_matching_responses(self, response_can_id, function_id):
        """
        Elimina del buffer cualquier mensaje viejo que coincida con el CAN ID de respuesta
        y function_id. Esto evita que respuestas tardías de comandos anteriores
        sean confundidas con la respuesta del comando actual.
        """
        with self._messages_lock:
            self._messages = [
                (msg, ts) for (msg, ts) in self._messages
                if not (msg.arbitration_id == response_can_id and 
                        len(msg.data) > 0 and msg.data[0] == function_id)
            ]

    def send_message(self, can_id, data, wait_for_reply=True, max_retries=None):
        """
        Envía un mensaje CAN y espera una respuesta del mismo CAN ID.
        Thread-safe: usa lock para envío y busca respuesta en el buffer (no en el bus).
        
        Args:
            can_id: ID del mensaje CAN
            data: Datos a enviar
            wait_for_reply: Si se debe esperar una respuesta
            
        Returns:
            Tupla (status, reply_data) donde:
                status: 'success', 'error', o 'not_found'
                reply_data: Datos de respuesta si es exitoso, None en caso contrario
        """
        try:
            response_can_id = can_id + 0x400
            function_id = data[0]
            
            # Limpiar respuestas viejas que coincidan ANTES de enviar
            self._purge_matching_responses(response_can_id, function_id)
            
            message = can.Message(
                arbitration_id=can_id,
                data=bytes(data),
                is_extended_id=False
            )
            
            with self._send_lock:
                self.channel.send(message)
            
            if not wait_for_reply:
                return 'success', None
            
            # Espera la respuesta (ya no necesitamos sent_after porque purgamos antes)
            reply_status, reply_data = self.read_message(30, response_can_id, function_id)
            
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
        Busca un mensaje específico en el buffer interno, NO en el bus.
        El hilo lector es el único que lee del bus; las respuestas se consumen aquí.
        
        Args:
            timeout_seconds: Tiempo máximo de espera
            search_can_id: CAN ID a buscar
            function_id: ID de función esperado en data[0]
            
        Returns a tuple (status, reply_data).
        """
        deadline = time.time() + timeout_seconds

        try:
            while time.time() < deadline:
                with self._messages_lock:
                    now = time.time()
                    i = 0
                    while i < len(self._messages):
                        msg, enqueued_ts = self._messages[i]

                        # Descartar mensajes expirados por TTL
                        if now - enqueued_ts > self._message_ttl:
                            self._messages.pop(i)
                            continue

                        if msg.arbitration_id == search_can_id:
                            can_data = list(msg.data)
                            if len(can_data) > 1 and function_id == can_data[0]:
                                # Extraemos este mensaje del buffer y retornamos
                                self._messages.pop(i)
                                if can_data[1] == 1:
                                    print(f"Valid response (success) from CAN ID {search_can_id}: {can_data}")
                                    return 'success', can_data
                                elif can_data[1] in [2, 3, 4]:
                                    print(f"Valid response (error) from CAN ID {search_can_id}: {can_data}")
                                    return 'error', can_data
                                # Si el contenido no indica éxito/error, seguimos buscando
                                continue

                        i += 1

                # Si no se encontró mensaje, esperar un poco antes de reintentar
                time.sleep(0.01)

            print(f"No valid response from CAN ID {search_can_id} within {timeout_seconds}s.")
            return 'not_found', None

        except Exception as e:
            print(f"Error in read_message: {e}")
            return 'not_found', None

    def _cleaner_loop(self):
        """Hilo de mantenimiento que elimina mensajes antiguos del buffer."""
        print("CAN queue cleaner thread started.")
        while not self._stop_cleaner_event.is_set():
            try:
                now = time.time()
                with self._messages_lock:
                    # Conservamos solo los mensajes dentro del TTL
                    self._messages = [
                        (msg, ts) for (msg, ts) in self._messages
                        if now - ts <= self._message_ttl
                    ]
            except Exception as e:
                print(f"Error in cleaner thread: {e}")

            time.sleep(5.0)
        print("CAN queue cleaner thread stopped.")

    def close_canbus(self):
        """Cierra la comunicación CAN y detiene el hilo lector."""
        if self.is_started:
            self._stop_reader_event.set()
            if self._reader_thread and self._reader_thread.is_alive():
                self._reader_thread.join(timeout=1)

            self._stop_cleaner_event.set()
            if self._cleaner_thread and self._cleaner_thread.is_alive():
                self._cleaner_thread.join(timeout=1)

            if self.channel:
                self.channel.shutdown()
            
            self.is_started = False
            print("CAN bus and reader thread stopped.")


# Ejemplo de uso
if __name__ == "__main__":
    # Crear instancia de Canbus
    can_bus = Canbus(bitrate=500000)  # Usar 500kbps como ejemplo
    
    # Iniciar comunicación
    if can_bus.start_canbus():
        print("CAN bus iniciado correctamente")
        
        try:
            # Ejemplo de envío de mensaje
            can_id = 0x123  # ID de ejemplo
            data = [0x01, 0x02, 0x03, 0x04]  # Datos de ejemplo
            
            print(f"Enviando mensaje a CAN ID: {can_id}, Datos: {data}")
            status, response = can_bus.send_message(can_id, data, wait_for_reply=True)
            
            print(f"Resultado: {status}")
            if response:
                print(f"Respuesta: {response}")
                
            # Ejemplo de recepción continua (descomentar para usar)
            # print("Esperando mensajes CAN (Ctrl+C para salir)...")
            # while True:
            #     msg = can_bus.channel.recv(1.0)  # Timeout de 1 segundo
            #     if msg:
            #         print(f"Mensaje recibido - ID: {msg.arbitration_id}, Datos: {list(msg.data)}")
                    
        except KeyboardInterrupt:
            print("\nOperación interrumpida por el usuario")
        finally:
            # Cerrar comunicación
            can_bus.close_canbus()
            print("CAN bus cerrado")
    else:
        print("Error al iniciar CAN bus")