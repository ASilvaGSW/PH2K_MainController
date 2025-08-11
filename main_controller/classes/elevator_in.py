# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Move Individual Actuator
#   IN: [actuator_id, angle_H, angle_L, orientation] | OUT: [0x03, status, ...]
# 0x04: Move Gantry XYZ
#   IN: [x_H, x_L, y_H, y_L, z_H, z_L, orientation] | OUT: [0x04, status, ...]
# 0x05: Move Elevator Z
#   IN: [angle_H, angle_L, orientation] | OUT: [0x05, status, ...]
# 0x06: Move Gripper
#   IN: [action: 0x01=open, 0x02=close] | OUT: [0x06, 0x01, ...]
# 0x07: Set gripper force
#   IN: [force (4-7)] | OUT: [0x07, 0x01, ...] if valid
# 0x08: Get gripper force
#   IN: None | OUT: [0x08, gripperForce, ...]
# 0x09: Get gripper counter
#   IN: None | OUT: [0x09, counter_H, counter_L, ...]
# 0x0A: Reset gripper counter
#   IN: None | OUT: [0x0A, 0x01, ...]
# 0x0B: Get Actuator Counter
#   IN: [actuator_id] | OUT: [0x0B, counter_H, counter_L, ...]
# 0x0C: Reset Actuator Counter
#   IN: [actuator_id] | OUT: [0x0C, 0x01, ...]
# 0xFF: Power off - Move all to home position
#   IN: None | OUT: None (moves all to home)

class ElevatorIn:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        status = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)[0]
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        status = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7)[0]
        return status

    # 0x03: Move Individual Actuator
    def move_individual_actuator(self, actuator_id, angle):

        orientation = 0

        if angle < 0 :
            orientation = 1
            angle = abs(angle)
        
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x03, actuator_id, angle_high, angle_low, orientation, 0x00, 0x00, 0x00])[0]
        return status

    def move_gantry_x(self,angle):
        return self.move_individual_actuator(1,angle)

    def move_gantry_y(self,angle):
        return self.move_individual_actuator(2,angle)

    def move_gantry_z(self,angle):
        return self.move_individual_actuator(4,angle)

    def move_gantry_array(self,data):
        return self.move_gantry_xyz(data[0],data[1],data[2])

    # 0x04: Move Gantry XYZ
    def move_gantry_xyz(self, x_angle, y_angle, z_angle):
     
        # Calculate orientation: 0b00000zyx, where z, y, x are 1 if negative
        orientation = 0
        if x_angle < 0:
            orientation |= 0b001
            x_angle = abs(x_angle)
        if y_angle < 0:
            orientation |= 0b010
            y_angle = abs(y_angle)
        if z_angle < 0:
            orientation |= 0b100
            z_angle = abs(z_angle)


        x_high = (x_angle >> 8) & 0xFF
        x_low = x_angle & 0xFF
        y_high = (y_angle >> 8) & 0xFF
        y_low = y_angle & 0xFF
        z_high = (z_angle >> 8) & 0xFF
        z_low = z_angle & 0xFF
        
        status = self.canbus.send_message(self.canbus_id, [0x04, x_high, x_low, y_high, y_low, z_high, z_low, orientation])[0]
        return status

    # 0x05: Move Elevator Z
    def move_elevator_z(self, angle):

        orientation = 0

        if angle < 0 :
            orientation = 1
            angle = abs(angle)
        
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, angle_high, angle_low, orientation, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x06: Move Gripper
    def move_gripper(self, action):
        status = self.canbus.send_message(self.canbus_id, [0x06, action, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    def open_gripper(self):
        return self.move_gripper(0x01)

    def close_gripper(self):
        return self.move_gripper(0x02)

    # 0x07: Set gripper force
    def set_gripper_force(self, force):
        status = self.canbus.send_message(self.canbus_id, [0x07, force, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0x08: Get gripper force
    def get_gripper_force(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)
        if status == 'success':
            gripper_force = reply_data[1]
            return status, gripper_force
        return status, None

    # 0x09: Get gripper counter
    def get_gripper_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        if status == 'success':
            counter = (reply_data[1] << 8) | reply_data[2]
            return status, counter
        return status, None

    # 0x0A: Reset gripper counter
    def reset_gripper_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x0A] + [0x00]*7)[0]
        return status

    # 0x0B: Get Actuator Counter
    def get_actuator_counter(self, actuator_id):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, actuator_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status == 'success':
            counter = (reply_data[1] << 8) | reply_data[2]
            return status, counter
        return status, None

    def get_x_counter(self):
        return self.get_actuator_counter(1)

    def get_y_counter(self):
        return self.get_actuator_counter(2)

    def get_z_counter(self):
        return self.get_actuator_counter(4)

    # 0x0C: Reset Actuator Counter
    def reset_actuator_counter(self, actuator_id):
        status = self.canbus.send_message(self.canbus_id, [0x0C, actuator_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])[0]
        return status

    # 0xFF: Power off - Move all to home position
    def power_off(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)[0]
        return status