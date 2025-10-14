# STAMPER 0x005
# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Home actuators (Y and Z)
#   IN: None | OUT: [0x03, status, ...]
# 0x04: Move Y actuator to absolute position
#   IN: [position_H, position_L, orientation] | OUT: [0x04, status, ...]
# 0x05: Move Z actuator to absolute position
#   IN: [position_H, position_L, orientation] | OUT: [0x05, status, ...]
# 0x06: Read Z actuator movement counter
#   IN: None | OUT: [0x06, 0x01, counter_H, counter_L, ...]
# 0x07: Read Y actuator movement counter
#   IN: None | OUT: [0x07, 0x01, counter_H, counter_L, ...]
# 0x08: Reset Y movement counter
#   IN: None | OUT: [0x08, 0x01, ...]
# 0x09: Reset Z movement counter
#   IN: None | OUT: [0x09, 0x01, ...]
# 0x0A: Read Y-axis stepper movement counter
#   IN: None | OUT: [0x0A, 0x01, counter_H, counter_L, ...]
# 0x0B: Move Y-axis stepper to position
#   IN: [position_H, position_L] | OUT: [0x0B, status, ...]
# 0x0C: Home Y-axis stepper
#   IN: None | OUT: [0x0C, status, ...]
# 0x12: Reset Y-axis stepper movement counter
#   IN: None | OUT: [0x12, 0x01, ...]
# 0x13: Home Y axis using go_home function
#   IN: None | OUT: [0x13, status, ...]
# 0x14: Home Z axis using go_home function
#   IN: None | OUT: [0x14, status, ...]
# 0x15: Move Y actuator to absolute position with speed control
#   IN: [position_H, position_L, orientation, speed] | OUT: [0x15, status, ...]
# 0x16: Read Z-axis stepper movement counter
#   IN: None | OUT: [0x16, 0x01, counter_H, counter_L, ...]
# 0x17: Move Z-axis stepper to position
#   IN: [position_H, position_L] | OUT: [0x17, status, ...]
# 0x18: Home Z-axis stepper
#   IN: None | OUT: [0x18, status, ...]
# 0x19: Reset Z-axis stepper movement counter
#   IN: None | OUT: [0x19, 0x01, ...]
# 0x1A: Control pump speed mode with timer (pump selection, direction, speed, time, acceleration)
#   IN: [pump_selection, direction, speed_H, speed_L, time_H, time_L, acceleration] | OUT: [0x1A, status, ...]
# 0x1B: Read pump movement counter (pump selection)
#   IN: [pump_selection] | OUT: [0x1B, 0x01, counter_H, counter_L, ...]
# 0x1C: Reset pump movement counter (pump selection)
#   IN: [pump_selection] | OUT: [0x1C, 0x01, ...]
# 0x1D: Control servo motors with angle positioning (servo selection, angle)
#   IN: [servo_selection, angle] | OUT: [0x1D, status, ...]
# 0x1E: Read servo movement counter (servo selection)
#   IN: [servo_selection] | OUT: [0x1E, 0x01, counter_H, counter_L, ...]
# 0x1F: Reset servo movement counter (servo selection)
#   IN: [servo_selection] | OUT: [0x1F, 0x01, ...]
# 0x20: Move stepper until optical sensor activated (stepper selection, sensor selection, direction, max steps)
#   IN: [stepper_selection, sensor_selection, direction, max_steps_H, max_steps_L] | OUT: [0x20, status, ...]
# 0x21: Read optical sensor counter (sensor selection)
#   IN: [sensor_selection] | OUT: [0x21, 0x01, counter_H, counter_L, ...]
# 0x22: Reset optical sensor counter (sensor selection)
#   IN: [sensor_selection] | OUT: [0x22, 0x01, ...]
# 0xFF: Power off
#   IN: None | OUT: [0xFF, 0x01, ...]

class Stamper:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # Case 0x01: Reset microcontroller
    def reset_microcontroller(self):
        """Reset the microcontroller"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01])
        return status

    # Case 0x02: Heartbeat
    def send_heartbeat(self):
        """Send heartbeat to check system status"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], max_retries=30)
        return status

    # Case 0x03: Home actuators (Y and Z)
    def home_actuators(self):
        """Home both Y and Z actuators"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x04: Move Y actuator to absolute position
    def move_y_actuator(self, position, orientation=0):
        """
        Move Y actuator to absolute position
        Args:
            position (int): Target position
            orientation (int): 0 for positive, 1 for negative
        """
        if position < 0:
            orientation = 1
            position = abs(position)
        
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x05: Move Z actuator to absolute position
    def move_z_actuator(self, position, orientation=0):
        """
        Move Z actuator to absolute position
        Args:
            position (int): Target position
            orientation (int): 0 for positive, 1 for negative
        """
        if position < 0:
            orientation = 1
            position = abs(position)
        
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x06: Read Z actuator movement counter
    def get_z_actuator_counter(self):
        """Get Z actuator movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x07: Read Y actuator movement counter
    def get_y_actuator_counter(self):
        """Get Y actuator movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x08: Reset Y movement counter
    def reset_y_actuator_counter(self):
        """Reset Y actuator movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x09: Reset Z movement counter
    def reset_z_actuator_counter(self):
        """Reset Z actuator movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x0A: Read Y-axis stepper movement counter
    def get_y_stepper_counter(self):
        """Get Y-axis stepper movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x0B: Move Y-axis stepper to position
    def move_y_stepper(self, position):
        """
        Move Y-axis stepper to position
        Args:
            position (int): Target position in steps
        """
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x0C: Home Y-axis stepper
    def home_y_stepper(self):
        """Home Y-axis stepper"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status



    # Case 0x12: Reset Y-axis stepper movement counter
    def reset_y_stepper_counter(self):
        """Reset Y-axis stepper movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x13: Home Y axis using go_home function
    def home_y_axis(self):
        """Home Y axis using go_home function"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x14: Home Z axis using go_home function
    def home_z_axis(self):
        """Home Z axis using go_home function"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x15: Move Y actuator to absolute position with speed control
    def move_y_actuator_with_speed(self, position, speed, orientation=0):
        """
        Move Y actuator to absolute position with speed control
        Args:
            position (int): Target position
            speed (int): Movement speed
            orientation (int): 0 for positive, 1 for negative
        """
        if position < 0:
            orientation = 1
            position = abs(position)
        
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15, position_high, position_low, orientation, speed, 0x00, 0x00, 0x00])
        return status

    # Case 0x16: Read Z-axis stepper movement counter
    def get_z_stepper_counter(self):
        """Get Z-axis stepper movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x17: Move Z-axis stepper to position
    def move_z_stepper(self, position):
        """
        Move Z-axis stepper to position
        Args:
            position (int): Target position in steps
        """
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x17, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x18: Home Z-axis stepper
    def home_z_stepper(self):
        """Home Z-axis stepper"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x19: Reset Z-axis stepper movement counter
    def reset_z_stepper_counter(self):
        """Reset Z-axis stepper movement counter"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x1A: Control pump speed mode with timer
    def control_pump(self, pump_selection, direction, speed, time_ms, acceleration=236):
        """
        Control pump speed mode with timer
        Args:
            pump_selection (int): Pump number (1-3)
            direction (int): Direction (0 or 1)
            speed (int): Speed value (0-65535)
            time_ms (int): Time in milliseconds (0-65535)
            acceleration (int): Acceleration value
        """
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        time_high = (time_ms >> 8) & 0xFF
        time_low = time_ms & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1A, pump_selection, direction, speed_high, speed_low, time_high, time_low, acceleration])
        return status

    # Case 0x1B: Read pump movement counter
    def get_pump_counter(self, pump_selection):
        """
        Get pump movement counter
        Args:
            pump_selection (int): Pump number (1-3)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1B, pump_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x1C: Reset pump movement counter
    def reset_pump_counter(self, pump_selection):
        """
        Reset pump movement counter
        Args:
            pump_selection (int): Pump number (1-3)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1C, pump_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x1D: Control servo motors with angle positioning
    def control_servo(self, servo_selection, angle):
        """
        Control servo motors with angle positioning
        Args:
            servo_selection (int): Servo number (1=cover, 2=stamper)
            angle (int): Target angle (0-180)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1D, servo_selection,angle, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x1E: Read servo movement counter
    def get_servo_counter(self, servo_selection):
        """
        Get servo movement counter
        Args:
            servo_selection (int): Servo number (1=cover, 2=stamper)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1E, servo_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x1F: Reset servo movement counter
    def reset_servo_counter(self, servo_selection):
        """
        Reset servo movement counter
        Args:
            servo_selection (int): Servo number (1=cover, 2=stamper)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1F, servo_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x20: Move stepper until optical sensor activated
    def move_stepper_until_sensor(self, stepper_selection, sensor_selection, direction, max_steps):
        """
        Move stepper until optical sensor activated
        Args:
            stepper_selection (int): Stepper selection (1=Y-axis, 2=Z-axis)
            sensor_selection (int): Sensor selection (1 or 2)
            direction (int): Direction (0 or 1)
            max_steps (int): Maximum steps to move
        """
        max_steps_high = (max_steps >> 8) & 0xFF
        max_steps_low = max_steps & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x20, stepper_selection, sensor_selection, direction, max_steps_high, max_steps_low, 0x00, 0x00])
        return status

    # Case 0x21: Read optical sensor counter
    def get_optical_sensor_counter(self, sensor_selection):
        """
        Get optical sensor counter
        Args:
            sensor_selection (int): Sensor number (1 or 2)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x21, sensor_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # Case 0x22: Reset optical sensor counter
    def reset_optical_sensor_counter(self, sensor_selection):
        """
        Reset optical sensor counter
        Args:
            sensor_selection (int): Sensor number (1 or 2)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x22, sensor_selection, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0xFF: Power off, home all axes
    def power_off(self):
        """Power off and home all axes"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Convenience methods for specific operations
    def home_all(self):
        """Home all actuators and steppers"""
        results = []
        results.append(self.home_actuators())
        results.append(self.home_y_stepper())
        results.append(self.home_z_stepper())
        return all(results)

    def move_servo_cover(self, angle):
        """Move servo cover to specific angle"""
        return self.control_servo(1, angle)

    def move_servo_stamper(self, angle):
        """Move servo stamper to specific angle"""
        return self.control_servo(2, angle)

    def control_pump1(self, direction, speed, time_ms, acceleration=100):
        """Control pump 1"""
        return self.control_pump(1, direction, speed, time_ms, acceleration)

    def control_pump2(self, direction, speed, time_ms, acceleration=100):
        """Control pump 2"""
        return self.control_pump(2, direction, speed, time_ms, acceleration)

    def control_pump3(self, direction, speed, time_ms, acceleration=100):
        """Control pump 3"""
        return self.control_pump(3, direction, speed, time_ms, acceleration)

    def reset_all_counters(self):
        """Reset all movement counters"""
        results = []
        results.append(self.reset_y_actuator_counter())
        results.append(self.reset_z_actuator_counter())
        results.append(self.reset_y_stepper_counter())
        results.append(self.reset_z_stepper_counter())
        results.append(self.reset_gripper_counter())
        for i in range(1, 4):  # Pumps 1-3
            results.append(self.reset_pump_counter(i))
        for i in range(1, 3):  # Servos 1-2
            results.append(self.reset_servo_counter(i))
        for i in range(1, 3):  # Optical sensors 1-2
            results.append(self.reset_optical_sensor_counter(i))
        return all(results)

    def get_all_counters(self):
        """
        Get all movement counters
        Returns:
            dict: Dictionary with all counter values
        """
        counters = {}
        
        # Actuator counters
        status, counter = self.get_y_actuator_counter()
        counters['y_actuator'] = counter if status else 0
        
        status, counter = self.get_z_actuator_counter()
        counters['z_actuator'] = counter if status else 0
        
        # Stepper counters
        status, counter = self.get_y_stepper_counter()
        counters['y_stepper'] = counter if status else 0
        
        status, counter = self.get_z_stepper_counter()
        counters['z_stepper'] = counter if status else 0
        
        # Pump counters
        for i in range(1, 4):
            status, counter = self.get_pump_counter(i)
            counters[f'pump{i}'] = counter if status else 0
        
        # Servo counters
        for i in range(1, 3):
            status, counter = self.get_servo_counter(i)
            servo_name = 'cover' if i == 1 else 'stamper'
            counters[f'servo_{servo_name}'] = counter if status else 0
        
        # Optical sensor counters
        for i in range(1, 3):
            status, counter = self.get_optical_sensor_counter(i)
            counters[f'optical_sensor_{i}'] = counter if status else 0
        
        return counters