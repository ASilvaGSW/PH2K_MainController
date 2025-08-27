import can
import time
import subprocess
import os

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
                    print(f"Ejecutando: {cmd}")
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

    def flush_buffer(self, timeout_ms=1):
        """
        Limpia cualquier mensaje pendiente en el buffer
        
        Args:
            timeout_ms: Tiempo de espera en milisegundos
        """
        try:
            timeout_sec = timeout_ms / 1000.0
            while True:
                msg = self.channel.recv(timeout=timeout_sec)
                if msg is None:  # Si no hay más mensajes
                    break
        except:
            pass  # Si hay timeout, el buffer está limpio

    def send_message(self, can_id, data, wait_for_reply=True, max_retries=None):
        """
        Envía un mensaje CAN y espera una respuesta del mismo CAN ID.
        
        Args:
            can_id: ID del mensaje CAN
            data: Datos a enviar
            wait_for_reply: Si se debe esperar una respuesta
            
        Returns:
            Tupla (status, reply_data) donde:
                status: 'success', 'error', o 'not_found'
                reply_data: Datos de respuesta si es exitoso, None en caso contrario
        """
        print("\n")
        
        try:
            # Limpia el buffer antes de enviar el nuevo comando
            self.flush_buffer()
            
            # Crear mensaje CAN
            message = can.Message(
                arbitration_id=can_id,
                data=bytes(data),
                is_extended_id=False
            )
            
            # Enviar el mensaje
            self.channel.send(message)
            
            if not wait_for_reply:
                return 'success', None
            
            # Espera la respuesta usando el nuevo read_message
            reply_status, reply_data = self.read_message(10, can_id + 0x400, data[0], max_retries)
            
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

    def read_message(self, timeout_ms, search_can_id, function_id, max_retries=None):
        """
        Lee un mensaje de un CAN ID específico después de enviar un mensaje.
        
        Args:
            timeout_ms: Tiempo de espera en milisegundos
            search_can_id: ID CAN a buscar
            max_retries: Número máximo de reintentos
            
        Returns:
            Tupla (status, reply_data) donde:
                status: 'success', 'error', o 'not_found'
                reply_data: Datos de respuesta si es exitoso, None en caso contrario
        """
        if max_retries is None:
            max_retries = 110  # 7 segundos si timeout_ms=100ms

        print(f"Starting read_message: timeout_ms={timeout_ms}, max_retries={max_retries}, expected_total_time={max_retries*timeout_ms}ms")
        start_time = time.time()
        found_id = False
        timeout_sec = timeout_ms / 1000.0

        try:
            for attempt in range(max_retries):
                iteration_start = time.time()
                try:
                    # Recibir mensaje con timeout
                    msg = self.channel.recv(timeout=timeout_sec)
                    
                    if msg is not None and msg.arbitration_id == search_can_id:
                        found_id = True
                        can_data = list(msg.data)
                        
                        if len(can_data) > 1 and function_id == can_data[0]:
                            if can_data[1] == 1 or can_data[1] == 3:
                                elapsed = time.time() - start_time
                                print(f"Valid response (success) from CAN ID {search_can_id}: {can_data} (found after {elapsed:.3f}s)")
                                return 'success', can_data
                            elif can_data[1] == 2 or can_data[1] == 4:
                                elapsed = time.time() - start_time
                                print(f"Valid response (error) from CAN ID {search_can_id}: {can_data} (found after {elapsed:.3f}s)")
                                return 'error', can_data
                except can.CanError as read_exception:
                    if "timeout" in str(read_exception).lower():
                        pass
                    else:
                        print(f"Read error on attempt {attempt + 1}: {read_exception}")
                except Exception as e:
                    print(f"Unexpected error on attempt {attempt + 1}: {e}")
                    
                time.sleep(0.1)
                iteration_time = time.time() - iteration_start
                if attempt % 10 == 0:
                    print(f"Attempt {attempt}: iteration took {iteration_time*1000:.1f}ms")
                    
            elapsed = time.time() - start_time
            if not found_id:
                print(f"No message received from CAN ID {search_can_id} after {max_retries} attempts (took {elapsed:.3f}s)")
                return 'not_found', None
            else:
                print(f"No valid response (success/error) from CAN ID {search_can_id} after {max_retries} attempts (took {elapsed:.3f}s)")
                return 'not_found', None
                
        except KeyboardInterrupt:
            elapsed = time.time() - start_time
            print(f"\nOperation interrupted by user after {elapsed:.3f}s")
            return 'not_found', None
        except Exception as e:
            elapsed = time.time() - start_time
            print(f"Error Reading Message after {elapsed:.3f}s: {e}")
            return 'not_found', None

    def close_canbus(self):
        """
        Cierra la comunicación CAN.
        """
        if self.channel:
            self.channel.shutdown()
            self.is_started = False


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