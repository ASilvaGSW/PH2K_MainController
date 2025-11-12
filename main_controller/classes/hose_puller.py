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
# 0x15: Move Y actuator to absolute position with speed control
# 0x16: Move Y axis with speed mode until hose presence sensor detects no hose
# 0x17: Move Y actuator to relative position with speed and acceleration control
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
        status = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],max_retries=30)[0]
        return status

    # Case 0x03: Home actuators (Y and Z)
    def home_actuators(self):
        status = self.canbus.send_message(self.canbus_id, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x04: Move Y actuator to absolute position
    def move_y_actuator(self, position):
        orientation = 1 if position < 0 else 0
        abs_position = abs(position)
        position_high = abs_position >> 8
        position_low = abs_position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x04, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # Case 0x05: Move Z actuator to absolute position
    def move_z_actuator(self, position):
        orientation = 1 if position < 0 else 0
        abs_position = abs(position)
        position_high = abs_position >> 8
        position_low = abs_position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00])[0]
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

    # Case 0x15: Move Y actuator to absolute position with speed control
    def move_y_actuator_with_speed(self, position, speed):
        orientation = 1 if position < 0 else 0
        abs_position = abs(position)
        position_high = abs_position >> 8
        position_low = abs_position & 0xFF
        speed_high = speed >> 8
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x15, position_high, position_low, orientation, speed_high, speed_low, 0x00, 0x00])[0]
        return status

    # Case 0x16: Move Y axis with speed mode until hose presence sensor detects no hose
    def move_y_axis_until_no_hose(self, speed, direction=0, acceleration=236):
        """
        Move Y axis in speed mode until hose presence sensor detects no hose.
        
        Args:
            speed (int): Speed value (0-65535)
            direction (int): Direction (0 = positive, 1 = negative)
            acceleration (int): Acceleration parameter (0-255)
            
        Returns:
            tuple: (status, hose_detected) where:
                - status: 'success', 'fail', 'timeout', or 'no_local_network'
                - hose_detected: True if hose still detected, False if no hose detected
        """
        speed_high = speed >> 8
        speed_low = speed & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16, speed_high, speed_low, direction, acceleration, 0x00, 0x00, 0x00])
        
        if status == 'success' and reply_data:
            # reply_data[2] contains sensor state: 1 = no hose detected, 0 = hose still detected
            hose_detected = not bool(reply_data[2])
            return status, hose_detected
        return status, None

    # Case 0x17: Move Y actuator to relative position with speed and acceleration control
    def move_y_actuator_relative_with_speed(self, relative_position, speed=100, acceleration=236):
        """
        Move Y actuator by a relative amount using speed and acceleration control (F4 frame).

        Args:
            relative_position (int): Relative angle/position (signed 16-bit range).
            speed (int): Speed in RPM (0-3000 recommended).
            acceleration (int): Acceleration (0-255), default 2.

        Returns:
            status (str): 'success', 'fail', 'timeout', or 'no_local_network'.
        """
        orientation = 1 if relative_position < 0 else 0
        abs_position = abs(relative_position)
        position_high = abs_position >> 8
        position_low = abs_position & 0xFF

        # Compose speed bytes
        speed_high = speed >> 8
        speed_low = speed & 0xFF

        # Acceleration constrained to one byte
        acc_byte = acceleration & 0xFF

        status = self.canbus.send_message(
            self.canbus_id,
            [0x17, position_high, position_low, orientation, speed_high, speed_low, acc_byte, 0x00]
        )[0]
        
        return status

    

    # Case 0xFF: Power off, home all axes
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status