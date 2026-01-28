
class PickAndPlaceCamera:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)[0]
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        status = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7, max_retries=30)[0]
        return status

    # 0x03: Load Model Nozzle
    def load_model_nozzle(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7, max_retries=500)
        if status == 'success' and data and len(data) >= 2:
            return data[1] == 0x01
        return False

    # 0x04: Load Model Joint
    def load_model_joint(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x04] + [0x00]*7, max_retries=500)
        if status == 'success' and data and len(data) >= 2:
            return data[1] == 0x01
        return False

    # 0x05: Count Objects Nozzle
    def count_objects_nozzle(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x05] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            if data[1] == 0xFF: return -1 # Error
            return data[1]
        return -1

    # 0x06: Count Objects Joint
    def count_objects_joint(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            if data[1] == 0xFF: return -1 # Error
            return data[1]
        return -1

    # 0x07: Alignment Nozzle
    def alignment_nozzle(self):
        # Alignment can take time, so we increase retries significantly
        status, data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7, max_retries=5000)
        if status == 'success' and data and len(data) >= 3:
            if data[1] == 0xFF and data[2] == 0xFF: return None # Error/Not found
            if data[1] == 0xFF and data[2] == 0xFE: return None # Model not loaded
            
            # Combine bytes to signed int
            distance = int.from_bytes([data[1], data[2]], byteorder='big', signed=True)
            return distance
        return None

    # 0x08: Alignment Joint
    def alignment_joint(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7, max_retries=5000)
        if status == 'success' and data and len(data) >= 3:
            if data[1] == 0xFF and data[2] == 0xFF: return None
            if data[1] == 0xFF and data[2] == 0xFE: return None
            
            distance = int.from_bytes([data[1], data[2]], byteorder='big', signed=True)
            return distance
        return None

    # 0x09: Pick Up Nozzle
    def pick_up_nozzle(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            if data[1] == 0xFF: return None # Error
            return data[1] # Returns the byte value mask
        return None

    # 0x10: Pick Up Joint
    def pick_up_joint(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7)
        if status == 'success' and data and len(data) >= 3:
            if data[1] == 0xFF: return None # Error
            # Combine bytes for 16-bit mask
            mask = int.from_bytes([data[1], data[2]], byteorder='big', signed=False)
            return mask
        return None

    # 0x11: Inspection Camera 0 (Nozzle)
    def inspection_camera_0(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            if data[1] == 0xFF: return False # Error
            return data[1] == 0x01
        return False

    # 0x12: Inspection Camera 1 (Joint)
    def inspection_camera_1(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            if data[1] == 0xFF: return False # Error
            return data[1] == 0x01
        return False

    # 0x13: Show Frame Nozzle
    def show_frame_nozzle(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x13] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            return data[1] == 0x01
        return False

    # 0x14: Show Frame Joint
    def show_frame_joint(self):
        status, data = self.canbus.send_message(self.canbus_id, [0x14] + [0x00]*7)
        if status == 'success' and data and len(data) >= 2:
            return data[1] == 0x01
        return False
