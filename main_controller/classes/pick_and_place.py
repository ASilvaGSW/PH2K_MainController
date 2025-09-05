# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x03: Home Actuators
#   IN: None | OUT: [0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x04: Move Actuator X to Absolute Position
#   IN: [pos_high, pos_low, orientation] | OUT: [0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x05: Move Actuator Z to Absolute Position
#   IN: [pos_high, pos_low, orientation] | OUT: [0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x06: Read Actuator Z Movement Counter
#   IN: None | OUT: [0x06, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
# 0x07: Read Actuator X Movement Counter
#   IN: None | OUT: [0x07, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
# 0x08: Reset Actuator X Movement Counter
#   IN: None | OUT: [0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x09: Reset Actuator Z Movement Counter
#   IN: None | OUT: [0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x0A: Move Left Conveyor in Speed Mode
#   IN: [direction, speed_high, speed_low, acceleration] | OUT: [0x0A, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x0B: Move Right Conveyor in Speed Mode
#   IN: [direction, speed_high, speed_low, acceleration] | OUT: [0x0B, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x0D: Open Gripper
#   IN: None | OUT: [0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x0E: Close Gripper
#   IN: None | OUT: [0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x0F: Set Gripper Force
#   IN: [force_value] | OUT: [0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x10: Read Gripper Movement Counter
#   IN: None | OUT: [0x10, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
# 0x11: Reset Gripper Movement Counter
#   IN: None | OUT: [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x12: Read Left Conveyor Movement Counter
#   IN: None | OUT: [0x12, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
# 0x13: Reset Left Conveyor Movement Counter
#   IN: None | OUT: [0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x14: Read Right Conveyor Movement Counter
#   IN: None | OUT: [0x14, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
# 0x15: Reset Right Conveyor Movement Counter
#   IN: None | OUT: [0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x16: Home X axis using go_home function
#   IN: None | OUT: [0x16, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x17: Home Z axis using go_home function
#   IN: None | OUT: [0x17, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x18: Move Left Conveyor Until IR Sensor Activation
#   IN: [direction, speed_high, speed_low, acceleration] | OUT: [0x18, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x19: Move Right Conveyor Until IR Sensor Activation
#   IN: [direction, speed_high, speed_low, acceleration] | OUT: [0x19, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x1A: Check Left IR Sensor Status
#   IN: None | OUT: [0x1A, 0x01, sensor_status, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x1B: Check Right IR Sensor Status
#   IN: None | OUT: [0x1B, 0x01, sensor_status, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x1C: Move Actuator X to Absolute Position with Speed Control
#   IN: [pos_high, pos_low, orientation, speed_high, speed_low] | OUT: [0x1C, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0x1D: Move Actuator Z to Absolute Position with Speed Control
#   IN: [pos_high, pos_low, orientation, speed_high, speed_low] | OUT: [0x1D, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# 0xFF: Power Off (Move All to Home Position)
#   IN: None | OUT: [0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

class PickAndPlace:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)[0]
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        status = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7,max_retries=30)[0]
        return status

    # 0x03: Home Actuators
    def home_actuators(self):
        status = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7)[0]
        return status

    # 0x04: Move Actuator X to Absolute Position
    def move_actuator_x(self, position, flag = True):
        orientation = 0
        if position < 0:
            orientation = 1
            position = abs(position)
        
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x04, pos_high, pos_low, orientation, 0x00, 0x00, 0x00, 0x00],flag)[0]
        return status

    # 0x05: Move Actuator Z to Absolute Position
    def move_actuator_z(self, position, flag = True):
        orientation = 0
        if position < 0:
            orientation = 1
            position = abs(position)
        
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, pos_high, pos_low, orientation, 0x00, 0x00, 0x00, 0x00],flag)[0]
        return status

    # 0x06: Read Actuator Z Movement Counter
    def get_z_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, None

    # 0x07: Read Actuator X Movement Counter
    def get_x_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, None

    # 0x08: Reset Actuator X Movement Counter
    def reset_x_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)[0]
        return status

    # 0x09: Reset Actuator Z Movement Counter
    def reset_z_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)[0]
        return status

    # 0x0A: Move Left Conveyor in Speed Mode
    def move_left_conveyor(self, direction, speed, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x0A, direction, speed_high, speed_low, acceleration, 0x00, 0x00, 0x00])[0]
        return status
    
    def start_left_conveyor(self,speed=1500):
        return self.move_left_conveyor(0, speed, 236)

    def stop_left_conveyor(self):
        return self.move_left_conveyor(0, 0, 0)

    # 0x0B: Move Right Conveyor in Speed Mode
    def move_right_conveyor(self, direction, speed, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x0B, direction, speed_high, speed_low, acceleration, 0x00, 0x00, 0x00])[0]
        return status

    def start_right_conveyor(self,speed=1500):
        return self.move_right_conveyor(0, speed, 236)

    def stop_right_conveyor(self):
        return self.move_right_conveyor(0, 0, 0)


    # 0x0D: Open Gripper
    def open_gripper(self):
        status = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7)[0]
        return status

    # 0x0E: Close Gripper
    def close_gripper(self):
        status = self.canbus.send_message(self.canbus_id, [0x0E] + [0x00]*7)[0]
        return status

    # 0x0F: Set Gripper Force
    def set_gripper_force(self, force):
        status = self.canbus.send_message(self.canbus_id, [0x0F, force, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x10: Read Gripper Movement Counter
    def get_gripper_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, None

    # 0x11: Reset Gripper Movement Counter
    def reset_gripper_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7)[0]
        return status

    # 0x12: Read Left Conveyor Movement Counter
    def get_left_conveyor_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, None

    # 0x13: Reset Left Conveyor Movement Counter
    def reset_left_conveyor_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x13] + [0x00]*7)[0]
        return status

    # 0x14: Read Right Conveyor Movement Counter
    def get_right_conveyor_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, None

    # 0x15: Reset Right Conveyor Movement Counter
    def reset_right_conveyor_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x15] + [0x00]*7)[0]
        return status

    # 0x16: Home X axis using go_home function
    def home_x_axis(self):
        status = self.canbus.send_message(self.canbus_id, [0x16] + [0x00]*7)[0]
        return status

    # 0x17: Home Z axis using go_home function
    def home_z_axis(self):
        status = self.canbus.send_message(self.canbus_id, [0x17] + [0x00]*7)[0]
        return status

    # 0x18: Move Left Conveyor Until IR Sensor Activation
    def move_left_conveyor_until_sensor(self, direction, speed, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x18, direction, speed_high, speed_low, acceleration, 0x00, 0x00, 0x00],max_retries=300)[0]
        return status

    # 0x19: Move Right Conveyor Until IR Sensor Activation
    def move_right_conveyor_until_sensor(self, direction, speed, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x19, direction, speed_high, speed_low, acceleration, 0x00, 0x00, 0x00],max_retries=300)[0]
        return status

    # 0x1A: Check Left IR Sensor Status
    def check_left_ir_sensor_status(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1A] + [0x00]*7)
        if status == 'success':
            sensor_status = reply_data[2]  # Third byte contains sensor status (0x01=object detected, 0x00=no object)
            return status, sensor_status
        return status, None

    # 0x1B: Check Right IR Sensor Status
    def check_right_ir_sensor_status(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1B] + [0x00]*7)
        if status == 'success':
            sensor_status = reply_data[2]  # Third byte contains sensor status (0x01=object detected, 0x00=no object)
            return status, sensor_status
        return status, None

    # 0x1C: Move Actuator X to Absolute Position with Speed Control
    def move_actuator_x_with_speed(self, position, speed, flag=True):
        orientation = 0
        if position < 0:
            orientation = 1
            position = abs(position)
        
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        
        status = self.canbus.send_message(self.canbus_id, [0x1C, pos_high, pos_low, orientation, speed_high, speed_low, 0x00, 0x00], flag)[0]
        return status

    # 0x1D: Move Actuator Z to Absolute Position with Speed Control
    def move_actuator_z_with_speed(self, position, speed, flag=True):
        orientation = 0
        if position < 0:
            orientation = 1
            position = abs(position)
        
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        
        status = self.canbus.send_message(self.canbus_id, [0x1D, pos_high, pos_low, orientation, speed_high, speed_low, 0x00, 0x00], flag)[0]
        return status

    # 0xFF: Power Off (Move All to Home Position)
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)[0]
        return status


