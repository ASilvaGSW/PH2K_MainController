# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x0D: Open all grippers
#   IN: None | OUT: [0x0D, status, ...]
# 0x0E: Close all grippers
#   IN: None | OUT: [0x0E, status, ...]
# 0x0F: Set all grippers force
#   IN: [force] | OUT: [0x0F, status, ...]
# 0x10: Read gripper 1 movement counter
#   IN: None | OUT: [0x10, status, counter_H, counter_L, ...]
# 0x11: Read gripper 2 movement counter
#   IN: None | OUT: [0x11, status, counter_H, counter_L, ...]
# 0x12: Read gripper 3 movement counter
#   IN: None | OUT: [0x12, status, counter_H, counter_L, ...]
# 0x13: Reset gripper 1 movement counter
#   IN: None | OUT: [0x13, status, ...]
# 0x14: Reset gripper 2 movement counter
#   IN: None | OUT: [0x14, status, ...]
# 0x15: Reset gripper 3 movement counter
#   IN: None | OUT: [0x15, status, ...]
# 0x16: Open gripper 1
#   IN: None | OUT: [0x16, status, ...]
# 0x17: Close gripper 1
#   IN: None | OUT: [0x17, status, ...]
# 0x18: Open gripper 2
#   IN: None | OUT: [0x18, status, ...]
# 0x19: Close gripper 2
#   IN: None | OUT: [0x19, status, ...]
# 0x1A: Open gripper 3
#   IN: None | OUT: [0x1A, status, ...]
# 0x1B: Close gripper 3
#   IN: None | OUT: [0x1B, status, ...]
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

    # 0x0D: Open all grippers
    def open_all_grippers(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7)
        return status

    # 0x0E: Close all grippers
    def close_all_grippers(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E] + [0x00]*7)
        return status

    # 0x0F: Set all grippers force
    def set_all_grippers_force(self, force):
        """
        Set force for all grippers
        :param force: Force value (0-255)
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0F, force] + [0x00]*6)
        return status

    # 0x10: Read gripper 1 movement counter
    def get_gripper1_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x11: Read gripper 2 movement counter
    def get_gripper2_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x12: Read gripper 3 movement counter
    def get_gripper3_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x13: Reset gripper 1 movement counter
    def reset_gripper1_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x13] + [0x00]*7)
        return status

    # 0x14: Reset gripper 2 movement counter
    def reset_gripper2_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x14] + [0x00]*7)
        return status

    # 0x15: Reset gripper 3 movement counter
    def reset_gripper3_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15] + [0x00]*7)
        return status

    # 0x16: Open gripper 1
    def open_gripper1(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16] + [0x00]*7)
        return status

    # 0x17: Close gripper 1
    def close_gripper1(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x17] + [0x00]*7)
        return status

    # 0x18: Open gripper 2
    def open_gripper2(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x18] + [0x00]*7)
        return status

    # 0x19: Close gripper 2
    def close_gripper2(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x19] + [0x00]*7)
        return status

    # 0x1A: Open gripper 3
    def open_gripper3(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1A] + [0x00]*7)
        return status

    # 0x1B: Close gripper 3
    def close_gripper3(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1B] + [0x00]*7)
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