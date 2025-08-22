# HOSE PULLER 0x192
# COMMANDS (CAN): 
# 0x01: Reset microcontroller
# 0x02: Heartbeat
# 0x03: Home actuators (Y and Z)
# 0x04: Move Y actuator to absolute position
# 0x05: Move Z actuator to absolute position
# 0x06: Read Z actuator movement counter
# 0x07: Read Y actuator movement counter
# 0x08: Reset Y movement counter
# 0x09: Reset Z movement counter
# 0x13: Home Y axis using go_home function
# 0x14: Home Z axis using go_home function
# 0xFF: Power off, home all axes.

class HosePuller:

    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # Case 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status = self.canbus.send_message(self.canbus_id, [0x01])[0]
        return status

    # Case 0x02: Heartbeat
    def send_heartbeat(self):
        status = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x03: Home actuators (Y and Z)
    def home_actuators(self):
        status = self.canbus.send_message(self.canbus_id, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x04: Move Y actuator to absolute position
    def move_y_actuator(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x04, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x05: Move Z actuator to absolute position
    def move_z_actuator(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x06: Read Z actuator movement counter
    def read_z_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = reply_data[2] << 8 | reply_data[3]
            return status, counter
        return status, None

    # Case 0x07: Read Y actuator movement counter
    def read_y_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = reply_data[2] << 8 | reply_data[3]
            return status, counter
        return status, None

    # Case 0x08: Reset Y movement counter
    def reset_y_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x09: Reset Z movement counter
    def reset_z_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x13: Home Y axis using go_home function
    def home_y_axis(self):
        status = self.canbus.send_message(self.canbus_id, [0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x14: Home Z axis using go_home function
    def home_z_axis(self):
        status = self.canbus.send_message(self.canbus_id, [0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0xFF: Power off, home all axes
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status