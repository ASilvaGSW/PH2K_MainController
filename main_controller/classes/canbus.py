import candle_driver
import time

class Canbus:

    def __init__(self, device_index: int = 0, channel_index: int = 0, bitrate: int = 125000):
        self.device_index = device_index
        self.channel_index = channel_index
        self.bitrate = bitrate
        self.device = None
        self.channel = None
        self.is_started = False

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
            self.is_started = True
            return True
        except Exception as e:
            print(f"Error Starting Canbus: {e}")
            self.is_started = False
            return False

    def flush_buffer(self, timeout_ms=1):
        """Limpia cualquier mensaje pendiente en el buffer"""
        try:
            while True:
                frame = self.channel.read(timeout_ms)
                if frame[0] is None:  # Si no hay más mensajes
                    break
        except:
            pass  # Si hay timeout, el buffer está limpio

    def send_message(self, can_id, data, wait_for_reply = True,max_retries=None):

        print("\n")

        """
        Send a CAN message and wait for a reply from the same CAN ID.
        Returns a tuple (status, reply_data) where:
            status: 'success', 'error', or 'not_found'
            reply_data: CAN data bytes if successful, None otherwise
        """
        try:
            # Limpia el buffer antes de enviar el nuevo comando
            self.flush_buffer()
            
            # Send the message
            result = self.channel.write(can_id, bytes(data))
            
            if result is None:
                print(f"Failed to send message to CAN ID {can_id}")
                return 'error', None
            
            if not wait_for_reply:
                return 'success', None
            
            # Espera la respuesta usando el nuevo read_message
            reply_status, reply_data = self.read_message(10, can_id + 0x400,max_retries,data[0])
            
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

    def read_message(self, timeout_ms, search_can_id, max_retries=None, function_id=0x00):
        """
        Lee un mensaje de un CAN ID específico después de enviar un mensaje.
        Returns a tuple (status, reply_data) where:
            status: 'success', 'error', or 'not_found'
            reply_data: CAN data bytes if successful, None otherwise
        """
        if max_retries is None:
            max_retries = 110  # 7 segundos si timeout_ms=100ms

        print(f"Starting read_message: timeout_ms={timeout_ms}, max_retries={max_retries}, expected_total_time={max_retries*timeout_ms}ms")
        start_time = time.time()
        found_id = False

        try:
            for attempt in range(max_retries):
                iteration_start = time.time()
                try:
                    frame_type, can_id, can_data, extended, ts = self.channel.read(timeout_ms)

                    if frame_type is not None and can_id is not None and can_id == search_can_id:
                        found_id = True
                        if len(can_data) > 1 and function_id == can_data[0]:
                            if can_data[1] == 1 or can_data[1] == 3:
                                elapsed = time.time() - start_time
                                print(f"Valid response (success) from CAN ID {search_can_id}: {can_data} (found after {elapsed:.3f}s)")
                                return 'success', can_data
                            elif can_data[1] == 2 or can_data[1] == 4:
                                elapsed = time.time() - start_time
                                print(f"Valid response (error) from CAN ID {search_can_id}: {can_data} (found after {elapsed:.3f}s)")
                                return 'error', can_data
                except Exception as read_exception:
                    if "timeout" in str(read_exception).lower():
                        pass
                    else:
                        print(f"Read error on attempt {attempt + 1}: {read_exception}")
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
        self.channel.stop()
        self.device.close()
        