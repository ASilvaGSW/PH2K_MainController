# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x04: Move X Axis (dual motors)
#   IN: [x_H, x_L, orientation] | OUT: [0x04, status, ...]
# 0x05: Move X Axis with Speed Control (dual motors)
#   IN: [x_H, x_L, orientation, speed_H, speed_L] | OUT: [0x05, status, ...]
# 0x06: Get Actuator Counter
#   IN: [actuator_id] | OUT: [0x06, counter_H, counter_L, ...] (ID: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3)
# 0x07: Reset Actuator Counter
#   IN: [actuator_id] | OUT: [0x07, 0x01, ...] (ID: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3)
# 0x08: Move All Steppers to Same Position
#   IN: [pos_H, pos_L, direction] | OUT: [0x08, status, ...]
# 0x09: Home X Axis
#   IN: None | OUT: [0x09, status, ...]
# 0xFF: Power off - Move X axis to home position
#   IN: None | OUT: None (moves X axis to home)

class TransporterFuyus:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7)
        return status

    # 0x04: Move X Axis (dual motors)
    def move_x_axis(self, angle):
        """
        Move X axis with dual motors
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

    # 0x05: Move X Axis with Speed Control (dual motors)
    def move_x_axis_with_speed(self, angle, speed=1000):
        """
        Move X axis with dual motors and speed control
        :param angle: Target angle (can be positive or negative)
        :param speed: Speed value (0-65535)
        """
        # Automatically calculate orientation based on angle sign
        orientation = 1 if angle < 0 else 0
        # Always use absolute value of angle
        abs_angle = abs(angle)
        
        angle_high = (abs_angle >> 8) & 0xFF
        angle_low = abs_angle & 0xFF
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, angle_high, angle_low, orientation, speed_high, speed_low] + [0x00]*2)
        return status

    # 0x06: Get Actuator Counter
    def get_actuator_counter(self, actuator_id):
        """
        Get counter for specific actuator
        :param actuator_id: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06, actuator_id] + [0x00]*6)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[1] << 8) | reply_data[2]
            return status, counter
        return status, 0

    # 0x07: Reset Actuator Counter
    def reset_actuator_counter(self, actuator_id):
        """
        Reset counter for specific actuator
        :param actuator_id: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07, actuator_id] + [0x00]*6)
        return status

    # 0x08: Move All Steppers to Same Position
    def move_all_steppers(self, position, direction=0):
        """
        Move all three steppers to the same position
        :param position: Target position (0-16777215) - 3 bytes
        :param direction: 0 = positive, 1 = negative
        """
        position_byte2 = (position >> 16) & 0xFF  # Most significant byte
        position_byte1 = (position >> 8) & 0xFF   # Middle byte
        position_byte0 = position & 0xFF          # Least significant byte
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08, position_byte2, position_byte1, position_byte0, direction] + [0x00]*3,300)
        return status

    # 0x09: Home X Axis
    def home_x_axis(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        return status

    # 0xFF: Power off - Move X axis to home position
    def power_off(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)
        return status

    # Convenience methods for specific actuator counters
    def get_x_axis_counter(self):
        return self.get_actuator_counter(1)

    def get_stepper1_counter(self):
        return self.get_actuator_counter(2)

    def get_stepper2_counter(self):
        return self.get_actuator_counter(3)

    def get_stepper3_counter(self):
        return self.get_actuator_counter(4)

    def reset_x_axis_counter(self):
        return self.reset_actuator_counter(1)

    def reset_stepper1_counter(self):
        return self.reset_actuator_counter(2)

    def reset_stepper2_counter(self):
        return self.reset_actuator_counter(3)

    def reset_stepper3_counter(self):
        return self.reset_actuator_counter(4)

    def moveToTapingHoseJig(self):
        return self.move_x_axis_with_speed(-265,100)

    def moveToStamperHoseJig(self):
        return self.move_x_axis_with_speed(-3530,100)
    
    def moveToReceivingPosition(self):
        return self.move_x_axis_with_speed(-5300,100)
    
    def moveToDeliverPosition(self):
        return self.move_x_axis_with_speed(350,100)
    
