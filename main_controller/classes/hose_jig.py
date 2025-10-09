# HOSE JIG 0x0CA
# COMMANDS (CAN): 
# 0x01: Reset microcontroller
# 0x02: Send ping
# 0x03: Home actuator
# 0x04: Move actuator to absolute position
# 0x05: Move servos to open position
# 0x06: Move servos to close position
# 0x08: Get servo movement counter
# 0x09: Reset servo movement counter
# 0x0A: Move servo to absolute position
# 0x0B: Move actuator to insertion position
# 0x0C: Move actuator to deliver position
# 0x0D: Read actuator movement counter
# 0x0E: Reset actuator movement counter
# 0x10: Update SERVO_OPEN_ANGLE
# 0x11: Update SERVO_CLOSE_ANGLE
# 0x12: Update ACTUATOR_DELIVER_POSITION
# 0x13: Update ACTUATOR_INSERTION_POSITION
# 0x14: Read SERVO_OPEN_ANGLE
# 0x15: Read SERVO_CLOSE_ANGLE
# 0x16: Read ACTUATOR_DELIVER_POSITION
# 0x17: Read ACTUATOR_INSERTION_POSITION
# 0x18: Home actuator using go_home function
# 0xFF: Power off - Move all to home position
 
class HoseJig:

    def __init__(self,canbus,canbus_id):

        self.canbus = canbus
        self.canbus_id = canbus_id


    #Case 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01])
        return status
    
    #Case 0x02: Send ping
    def send_heartbeat(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],max_retries=30)
        return status
    
    #Case 0x03: Home actuator
    def home(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x04: Move actuator to absolute position
    def move_actuator(self, position):

        if position < 0:
            orientation = 1
        else:
            orientation = 0

        position = abs(position)

        position_high = position >> 8
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x05: Move servos to open position
    def gripper_open(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x06: Move servos to close position
    def gripper_close(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x08: Get servo movement counter
    def gripper_counter(self):

        counter = reply_data[2] << 8 | reply_data[3]
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status, counter

    #Case 0x09: Reset servo movement counter
    def reset_gripper_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x0A: Move servo to absolute position
    def move_gripper(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x0B: Move actuator to insertion position
    def insertion_position(self,flag = True):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],flag)
        return status

    #Case 0x0C: Move actuator to deliver position
    def deliver_position(self,flag = True):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],flag)
        return status

    #Case 0x0D: Read actuator movement counter
    def actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        count = reply_data[2] << 8 | reply_data[3]
        return status, count

    
    #Case 0x0E: Reset actuator movement counter
    def reset_actuator_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x10: Update SERVO_OPEN_ANGLE
    def update_open(self, angle):
        angle_high = angle >> 8
        angle_low = angle & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x11: Update SERVO_CLOSE_ANGLE
    def update_close(self, angle):
        angle_high = angle >> 8
        angle_low = angle & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x11, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x12: Update ACTUATOR_DELIVER_POSITION
    def update_deliver(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x13: Update ACTUATOR_INSERTION_POSITION
    def update_insertion(self, position):
        position_high = position >> 8
        position_low = position & 0xFF
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x13, position_high, position_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0x14: Read Servo Open Angle
    def read_open(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        angle = reply_data[2] << 8 | reply_data[3]
        return status, angle

    #Case 0x15: Read Servo Close Angle
    def read_close(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        angle = reply_data[2] << 8 | reply_data[3]
        return status, angle

    #Case 0x16: Read Actuator Deliver Position
    def read_deliver(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        position = reply_data[2] << 8 | reply_data[3]
        return status, position

    #Case 0x17: Read Actuator Insertion Position
    def read_insertion(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        position = reply_data[2] << 8 | reply_data[3]
        return status, position

    #Case 0x18: Home actuator using go_home function
    def go_home(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Case 0xFF: Power off - Move all to home position
    def power_off(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status