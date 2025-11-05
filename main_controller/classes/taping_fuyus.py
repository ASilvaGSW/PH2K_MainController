# TAPING FUYUS 0x007
# COMMANDS (CAN): 
# 0x01: Reset microcontroller
# 0x02: Send Heartbeat
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
# 0x16: Move Z actuator to absolute position with speed control
# 0xFF: Power off - Move all to home position

class TapingFuyus:

    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # Case 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)
        return status
    
    # Case 0x02: Send Heartbeat
    def send_heartbeat(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7, max_retries=30)
        return status
    
    # Case 0x03: Home actuators (Y and Z)
    def home_all_actuators(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7)
        return status

    # Case 0x04: Move Y actuator to absolute position
    def move_y_actuator(self, angle):
        """
        Move Y actuator to absolute position
        :param angle: Target angle (can be positive or negative)
        """
        # Automatically calculate orientation based on angle sign
        orientation = 1 if angle < 0 else 0
        # Always use absolute value of angle
        abs_angle = abs(angle)
        
        angle_high = (abs_angle >> 8) & 0xFF
        angle_low = abs_angle & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04, angle_high, angle_low, orientation] + [0x00]*4)
        return status

    # Case 0x05: Move Z actuator to absolute position
    def move_z_actuator(self, angle):
        """
        Move Z actuator to absolute position
        :param angle: Target angle (can be positive or negative)
        """
        # Automatically calculate orientation based on angle sign
        orientation = 1 if angle < 0 else 0
        # Always use absolute value of angle
        abs_angle = abs(angle)
        
        angle_high = (abs_angle >> 8) & 0xFF
        angle_low = abs_angle & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, angle_high, angle_low, orientation] + [0x00]*4)
        return status

    # Case 0x06: Read Z actuator movement counter
    def get_z_actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        if status and reply_data and len(reply_data) >= 6:
            # Counter is returned as 4-byte little-endian in bytes 2-5
            counter = (reply_data[5] << 24) | (reply_data[4] << 16) | (reply_data[3] << 8) | reply_data[2]
            return status, counter
        return status, 0

    # Case 0x07: Read Y actuator movement counter
    def get_y_actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7)
        if status and reply_data and len(reply_data) >= 6:
            # Counter is returned as 4-byte little-endian in bytes 2-5
            counter = (reply_data[5] << 24) | (reply_data[4] << 16) | (reply_data[3] << 8) | reply_data[2]
            return status, counter
        return status, 0

    # Case 0x08: Reset Y movement counter
    def reset_y_actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)
        return status

    # Case 0x09: Reset Z movement counter
    def reset_z_actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        return status

    # Case 0x13: Home Y axis using go_home function
    def home_y_actuator(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x13] + [0x00]*7)
        return status

    # Case 0x14: Home Z axis using go_home function
    def home_z_actuator(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14] + [0x00]*7)
        return status

    # Case 0x15: Move Y actuator to absolute position with speed control
    def move_y_actuator_with_speed(self, angle, speed=100):
        """
        Move Y actuator to absolute position with speed control
        :param angle: Target angle (can be positive or negative)
        :param speed: Speed value (0-1500) encoded in 2 bytes (high/low)
        """
        # Automatically calculate orientation based on angle sign
        orientation = 1 if angle < 0 else 0
        # Always use absolute value of angle
        abs_angle = abs(angle)
        
        angle_high = (abs_angle >> 8) & 0xFF
        angle_low = abs_angle & 0xFF
        # English: Speed uses 2 bytes (0-1500). Español: Velocidad usa 2 bytes (0-1500). 日本語: 速度は2バイト(0-1500)
        speed_value = max(0, min(1500, int(speed)))
        speed_high = (speed_value >> 8) & 0xFF
        speed_low = speed_value & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15, angle_high, angle_low, orientation, speed_high, speed_low] + [0x00]*2)
        return status

    # Case 0x16: Move Z actuator to absolute position with speed control
    def move_z_actuator_with_speed(self, angle, speed=100):
        """
        Move Z actuator to absolute position with speed control
        :param angle: Target angle (can be positive or negative)
        :param speed: Speed value (0-1500) encoded in 2 bytes (high/low)
        """
        # Automatically calculate orientation based on angle sign
        orientation = 1 if angle < 0 else 0
        # Always use absolute value of angle
        abs_angle = abs(angle)
        
        angle_high = (abs_angle >> 8) & 0xFF
        angle_low = abs_angle & 0xFF
        # English: Speed uses 2 bytes (0-1500). Español: Velocidad usa 2 bytes (0-1500). 日本語: 速度は2バイト(0-1500)
        speed_value = max(0, min(1500, int(speed)))
        speed_high = (speed_value >> 8) & 0xFF
        speed_low = speed_value & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16, angle_high, angle_low, orientation, speed_high, speed_low] + [0x00]*2)
        return status

    # Case 0xFF: Power off - Move all to home position
    def power_off(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)
        return status

    # Convenience methods for common operations
    def home(self):
        """Home all actuators (alias for home_all_actuators)"""
        return self.home_all_actuators()

    def move_to_position(self, y_angle=None, z_angle=None):
        """
        Move both actuators to specified positions
        :param y_angle: Y actuator angle (None to skip)
        :param z_angle: Z actuator angle (None to skip)
        :return: Tuple of (y_status, z_status)
        """
        y_status = True
        z_status = True

        if z_angle is not None:
            z_status = self.move_z_actuator(z_angle)
        
        if y_angle is not None:
            y_status = self.move_y_actuator(y_angle)
        
        
        return y_status, z_status

    def get_all_counters(self):
        """
        Get both actuator counters
        :return: Tuple of ((y_status, y_counter), (z_status, z_counter))
        """
        y_status, y_counter = self.get_y_actuator_counter()
        z_status, z_counter = self.get_z_actuator_counter()
        return (y_status, y_counter), (z_status, z_counter)

    def reset_all_counters(self):
        """
        Reset both actuator counters
        :return: Tuple of (y_status, z_status)
        """
        y_status = self.reset_y_actuator_counter()
        z_status = self.reset_z_actuator_counter()
        return y_status, z_status