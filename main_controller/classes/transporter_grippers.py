# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Open all grippers
#   IN: None | OUT: [0x03, status, ...]
# 0x04: Close all grippers
#   IN: None | OUT: [0x04, status, ...]
# 0x05: Set all grippers force
#   IN: [force] | OUT: [0x05, status, ...]
# 0x06: Read gripper 1 movement counter
#   IN: None | OUT: [0x06, status, counter_H, counter_L, ...]
# 0x07: Read gripper 2 movement counter
#   IN: None | OUT: [0x07, status, counter_H, counter_L, ...]
# 0x08: Read gripper 3 movement counter
#   IN: None | OUT: [0x08, status, counter_H, counter_L, ...]
# 0x09: Reset gripper 1 movement counter
#   IN: None | OUT: [0x09, status, ...]
# 0x0A: Reset gripper 2 movement counter
#   IN: None | OUT: [0x0A, status, ...]
# 0x0B: Reset gripper 3 movement counter
#   IN: None | OUT: [0x0B, status, ...]
# 0x0C: Open gripper 1
#   IN: None | OUT: [0x0C, status, ...]
# 0x0D: Close gripper 1
#   IN: None | OUT: [0x0D, status, ...]
# 0x0E: Open gripper 2
#   IN: None | OUT: [0x0E, status, ...]
# 0x0F: Close gripper 2
#   IN: None | OUT: [0x0F, status, ...]
# 0x10: Open gripper 3
#   IN: None | OUT: [0x10, status, ...]
# 0x11: Close gripper 3
#   IN: None | OUT: [0x11, status, ...]
# 0xFF: Power off - Open all grippers
#   IN: None | OUT: [0xFF, status, ...]

class TransporterGrippers:
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

    # 0x03: Open all grippers
    def open_all_grippers(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7)
        return status

    # 0x04: Close all grippers
    def close_all_grippers(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04] + [0x00]*7)
        return status

    # 0x05: Set all grippers force
    def set_all_grippers_force(self, force):
        """
        Set force for all grippers
        :param force: Force value (0-255)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, force] + [0x00]*6)
        return status

    # 0x06: Read gripper 1 movement counter
    def get_gripper1_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x07: Read gripper 2 movement counter
    def get_gripper2_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x08: Read gripper 3 movement counter
    def get_gripper3_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x09: Reset gripper 1 movement counter
    def reset_gripper1_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        return status

    # 0x0A: Reset gripper 2 movement counter
    def reset_gripper2_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A] + [0x00]*7)
        return status

    # 0x0B: Reset gripper 3 movement counter
    def reset_gripper3_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B] + [0x00]*7)
        return status

    # 0x0C: Open gripper 1
    def open_gripper1(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C] + [0x00]*7)
        return status

    # 0x0D: Close gripper 1
    def close_gripper1(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7)
        return status

    # 0x0E: Open gripper 2
    def open_gripper2(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E] + [0x00]*7)
        return status

    # 0x0F: Close gripper 2
    def close_gripper2(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0F] + [0x00]*7)
        return status

    # 0x10: Open gripper 3
    def open_gripper3(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7)
        return status

    # 0x11: Close gripper 3
    def close_gripper3(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7)
        return status

    # 0xFF: Power off - Open all grippers
    def power_off(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)
        return status

    # Convenience methods for gripper operations
    def open_gripper(self, gripper_id):
        """
        Open specific gripper by ID
        :param gripper_id: 1, 2, or 3
        """
        if gripper_id == 1:
            return self.open_gripper1()
        elif gripper_id == 2:
            return self.open_gripper2()
        elif gripper_id == 3:
            return self.open_gripper3()
        else:
            raise ValueError("Invalid gripper_id. Must be 1, 2, or 3.")

    def close_gripper(self, gripper_id):
        """
        Close specific gripper by ID
        :param gripper_id: 1, 2, or 3
        """
        if gripper_id == 1:
            return self.close_gripper1()
        elif gripper_id == 2:
            return self.close_gripper2()
        elif gripper_id == 3:
            return self.close_gripper3()
        else:
            raise ValueError("Invalid gripper_id. Must be 1, 2, or 3.")

    def get_gripper_counter(self, gripper_id):
        """
        Get counter for specific gripper by ID
        :param gripper_id: 1, 2, or 3
        """
        if gripper_id == 1:
            return self.get_gripper1_counter()
        elif gripper_id == 2:
            return self.get_gripper2_counter()
        elif gripper_id == 3:
            return self.get_gripper3_counter()
        else:
            raise ValueError("Invalid gripper_id. Must be 1, 2, or 3.")

    def reset_gripper_counter(self, gripper_id):
        """
        Reset counter for specific gripper by ID
        :param gripper_id: 1, 2, or 3
        """
        if gripper_id == 1:
            return self.reset_gripper1_counter()
        elif gripper_id == 2:
            return self.reset_gripper2_counter()
        elif gripper_id == 3:
            return self.reset_gripper3_counter()
        else:
            raise ValueError("Invalid gripper_id. Must be 1, 2, or 3.")