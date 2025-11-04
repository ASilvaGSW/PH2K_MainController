# TAPING 0x00A
# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Execute step1 - Feeder
#   IN: None | OUT: [0x03, 0x01, ...]
# 0x04: Execute step2 - Cutter
#   IN: None | OUT: [0x04, 0x01, ...]
# 0x05: Execute step3 - Applicator
#   IN: None | OUT: [0x05, 0x01, ...]
# 0x06: Execute step4 - Holder
#   IN: None | OUT: [0x06, 0x01, ...]
# 0x07: Execute step5 - Holder Home
#   IN: None | OUT: [0x07, 0x01, ...]
# 0x08: Execute step6 - Applicator Home
#   IN: None | OUT: [0x08, 0x01, ...]
# 0x09: Execute step7 - Holder Home Position
#   IN: None | OUT: [0x09, 0x01, ...]
# 0x0A: Execute step8 - Gripper Close
#   IN: None | OUT: [0x0A, 0x01, ...]
# 0x0B: Execute step9 - Gripper Open
#   IN: None | OUT: [0x0B, 0x01, ...]
# 0x0C: Execute step10 - Elevator Down
#   IN: None | OUT: [0x0C, 0x01, ...]
# 0x0D: Execute step11 - Elevator Up
#   IN: None | OUT: [0x0D, 0x01, ...]
# 0x0E: Execute step12 - Feeder Reverse
#   IN: None | OUT: [0x0E, 0x01, ...]
# 0x0F: Execute step13 - Feeder Reverse (Alternative)
#   IN: None | OUT: [0x0F, 0x01, ...]
# 0x10: Execute FullCycle sequence
#   IN: None | OUT: [0x10, 0x01, ...]
# 0x11: Execute Forward sequence
#   IN: None | OUT: [0x11, 0x01, ...]
# 0x12: Execute Backward sequence
#   IN: None | OUT: [0x12, 0x01, ...]

class Taping:
    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

    # 0x01: Reset microcontroller
    def reset_microcontroller(self):
        """Reset the taping microcontroller"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01] + [0x00]*7)
        return status

    # 0x02: Send Heartbeat
    def send_heartbeat(self):
        """Send heartbeat to check taping device status"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02] + [0x00]*7)
        return status

    # 0x03: Execute step1 - Feeder
    def step1_feeder(self):
        """Execute step1 - Feed tape using servo1 with encoder feedback"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03] + [0x00]*7)
        return status

    # 0x04: Execute step2 - Cutter
    def step2_cutter(self):
        """Execute step2 - Wrapper 1/4 spin CCW direction"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04] + [0x00]*7)
        return status

    # 0x05: Execute step3 - Applicator
    def step3_applicator(self):
        """Execute step3 - Wrapper 1/2 spin CW direction"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05] + [0x00]*7)
        return status

    # 0x06: Execute step4 - Holder
    def step4_holder(self):
        """Execute step4 - Move holder to hold position"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        return status

    # 0x07: Execute step5 - Holder Home
    def step5_holder_home(self):
        """Execute step5 - Move holder to home position"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7)
        return status

    # 0x08: Execute step6 - Applicator Home
    def step6_applicator_home(self):
        """Execute step6 - Move applicator to home position"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)
        return status

    # 0x09: Execute step7 - Holder Home Position
    def step7_holder_home_position(self):
        """Execute step7 - Ensure holder is in home position"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)
        return status

    # 0x0A: Execute step8 - Gripper Close
    def step8_gripper_close(self):
        """Execute step8 - Close gripper to hold hose"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A] + [0x00]*7)
        return status

    # 0x0B: Execute step9 - Gripper Open
    def step9_gripper_open(self):
        """Execute step9 - Open gripper to release hose"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B] + [0x00]*7)
        return status

    # 0x0C: Execute step10 - Elevator Down
    def step10_elevator_down(self):
        """Execute step10 - Move elevator down and open gripper"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C] + [0x00]*7)
        return status

    # 0x0D: Execute step11 - Elevator Up
    def step11_elevator_up(self):
        """Execute step11 - Move elevator up"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7)
        return status

    # 0x0E: Execute step12 - Feeder Reverse
    def step12_feeder_reverse(self):
        """Execute step12 - Reverse feeder movement"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E] + [0x00]*7)
        return status

    # 0x0F: Execute step13 - Feeder Reverse (Alternative)
    def step13_feeder_reverse_alt(self):
        """Execute step13 - Alternative feeder reverse movement"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0F] + [0x00]*7)
        return status

    # 0x10: Execute FullCycle sequence
    def full_cycle(self):
        """Execute FullCycle sequence - Complete taping cycle"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x10] + [0x00]*7)
        return status

    # 0x11: Execute Forward sequence
    def forward_sequence(self):
        """Execute Forward sequence - Elevator down, gripper close, holder home, feeder reverse"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x11] + [0x00]*7)
        return status

    # 0x12: Execute Backward sequence
    def backward_sequence(self):
        """Execute Backward sequence - Cutter, applicator, holder, applicator home, holder home"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x12] + [0x00]*7)
        return status

    # Convenience methods for common operations
    def feed_tape(self):
        """Feed tape - alias for step1_feeder"""
        return self.step1_feeder()

    def cut_tape(self):
        """Cut tape - alias for step2_cutter"""
        return self.step2_cutter()

    def wrap_tape(self):
        """Wrap tape - alias for step3_applicator"""
        return self.step3_applicator()

    def hold_tape(self):
        """Hold tape - alias for step4_holder"""
        return self.step4_holder()

    def release_tape(self):
        """Release tape - alias for step5_holder_home"""
        return self.step5_holder_home()

    def close_gripper(self):
        """Close gripper - alias for step8_gripper_close"""
        return self.step8_gripper_close()

    def open_gripper(self):
        """Open gripper - alias for step9_gripper_open"""
        return self.step9_gripper_open()

    def elevator_up(self):
        """Move elevator up - alias for step11_elevator_up"""
        return self.step11_elevator_up()

    def elevator_down(self):
        """Move elevator down - alias for step10_elevator_down"""
        return self.step10_elevator_down()

    def reverse_feeder(self):
        """Reverse feeder - alias for step12_feeder_reverse"""
        return self.step12_feeder_reverse()

    # High-level operations
    def home_all(self):
        """Move all components to home position"""
        # Execute sequence to home all components
        self.step7_holder_home_position()
        self.step6_applicator_home()
        self.step9_gripper_open()
        return True

    def prepare_for_hose(self):
        """Prepare taping station for hose insertion"""
        self.elevator_down()
        self.open_gripper()
        return True

    def complete_taping_cycle(self):
        """Execute complete taping cycle"""
        return self.full_cycle()

    def power_off(self):
        """Power off - move all components to safe positions"""
        return self.home_all()