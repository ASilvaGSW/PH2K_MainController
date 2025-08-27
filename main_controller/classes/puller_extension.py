# PULLER EXTENSION 0x193
# COMMANDS (CAN): 
# 0x01: Reset microcontroller
# 0x02: Heartbeat
# 0x0A: Read stepper movement counter
# 0x0B: Move stepper to position
# 0x0C: Home stepper
# 0x0D: Open gripper
# 0x0E: Close gripper
# 0x0F: Set gripper force
# 0x10: Read gripper movement counter
# 0x11: Reset gripper movement counter
# 0x12: Reset stepper movement counter
# 0xFF: Power off, home all axes

class PullerExtension:

    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # Case 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status = self.canbus.send_message(self.canbus_id, [0x01])[0]
        return status

    # Case 0x02: Heartbeat
    def send_heartbeat(self):
        status = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],max_retries=30)[0]
        return status

    # Case 0x0A: Read stepper movement counter
    def read_stepper_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = reply_data[2] << 8 | reply_data[3]
            return status, counter
        return status, None

    # Case 0x0B: Move stepper to position
    def move_stepper(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x0B, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x0C: Home stepper
    def home_stepper(self):
        status = self.canbus.send_message(self.canbus_id, [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x0D: Open gripper
    def open_gripper(self):
        status = self.canbus.send_message(self.canbus_id, [0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x0E: Close gripper
    def close_gripper(self):
        status = self.canbus.send_message(self.canbus_id, [0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x0F: Set gripper force
    def set_gripper_force(self, force):
        force_high = force >> 8
        force_low = force & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x0F, force_high, force_low, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x10: Read gripper movement counter
    def read_gripper_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = reply_data[2] << 8 | reply_data[3]
            return status, counter
        return status, None

    # Case 0x11: Reset gripper movement counter
    def reset_gripper_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x12: Reset stepper movement counter
    def reset_stepper_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0xFF: Power off, home all axes
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status