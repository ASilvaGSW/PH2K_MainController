# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x05: Move Elevator Z
#   IN: [angle_H, angle_L, orientation] | OUT: [0x05, status, ...]
# 0x0B: Get Actuator Counter
#   IN: [actuator_id] | OUT: [0x0B, counter_H, counter_L, ...]
# 0x0C: Reset Actuator Counter
#   IN: [actuator_id] | OUT: [0x0C, 0x01, ...]
# 0x10: Home Individual Actuator
#   IN: [actuator_id] | OUT: [0x10, status, ...]
# 0x12: Home All Axes (Elevator Z)
#   IN: None | OUT: [0x12, status, ...]
# 0x13: Control Servo
#   IN: [pwm_H, pwm_L, mode] | OUT: [0x13, 0x01, ...]
# 0x14: Get Servo Counter
#   IN: [servo_id] | OUT: [0x14, counter_H, counter_L, ...]
# 0x15: Reset Servo Counter
#   IN: [servo_id] | OUT: [0x15, 0x01, ...]
# 0x16: Get Optical Sensor Status
#   IN: None | OUT: [0x16, sensor_status, ...]
# 0x17: Move Elevator to Reception
#   IN: None | OUT: [0x17, status, ...]
# 0x18: Set Servo State (Home/Open)
#   IN: [state, mode] | OUT: [0x18, 0x01, ...]
# 0x19: Move Elevator Relative (Jog)
#   IN: [angle_H, angle_L] | OUT: [0x19, status, ...]
# 0xFF: Power off
#   IN: None | OUT: None (moves servos to home)

class ElevatorOut:
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

    # 0x05: Move Elevator Z
    def move_elevator_z(self, angle):
        orientation = 0
        if angle < 0:
            orientation = 1
            angle = abs(angle)
        
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, angle_high, angle_low, orientation, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x0B: Get Actuator Counter
    def get_actuator_counter(self, actuator_id):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, actuator_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = (reply_data[1] << 8) | reply_data[2]
            return status, counter
        return status, None

    # 0x0C: Reset Actuator Counter
    def reset_actuator_counter(self, actuator_id):
        status = self.canbus.send_message(self.canbus_id, [0x0C, actuator_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x10: Home Individual Actuator
    def home_individual_actuator(self, actuator_id):
        status = self.canbus.send_message(self.canbus_id, [0x10, actuator_id] + [0x00]*6, max_retries=2000)[0]
        return status

    # 0x12: Home All Axes (Elevator Z)
    def home_all_axes(self):
        status = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7)[0]
        return status

    # 0x13: Control Servo
    def control_servo(self, pwm_value, mode=0):
        pwm_high = (pwm_value >> 8) & 0xFF
        pwm_low = pwm_value & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x13, pwm_high, pwm_low, mode, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    def open_servo(self, mode=0):
        # PWM value for open is 850 based on firmware comments
        return self.control_servo(850, mode)

    def release_servo(self, mode=0):
        # PWM value for release/home is 2500 based on firmware comments
        return self.control_servo(2500, mode)

    # 0x14: Get Servo Counter
    def get_servo_counter(self, servo_id=1):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14, servo_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = (reply_data[1] << 8) | reply_data[2]
            return status, counter
        return status, None

    # 0x15: Reset Servo Counter
    def reset_servo_counter(self, servo_id=1):
        status = self.canbus.send_message(self.canbus_id, [0x15, servo_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x16: Get Optical Sensor Status
    def get_optical_sensor_status(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16] + [0x00]*7)
        if status == 'success':
            sensor_status = reply_data[1]
            return status, sensor_status
        return status, None

    # 0x17: Move Elevator to Reception
    def move_elevator_to_reception(self):
        status = self.canbus.send_message(self.canbus_id, [0x17] + [0x00]*7)[0]
        return status

    # 0x18: Set Servo State (Home/Open)
    def set_servo_state(self, state, mode=0):
        if state == 1:  # Open
            return self.open_servo(mode)
        else:  # Home/Release
            return self.release_servo(mode)

    # 0x19: Move Elevator Relative (Jog)
    def move_elevator_relative(self, angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x19, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0xFF: Power off
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)[0]
        return status
