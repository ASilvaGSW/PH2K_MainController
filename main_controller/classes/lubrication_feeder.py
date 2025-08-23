# LUBRICATION FEEDER 0x019
# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Send Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Activate Valve 1 (Primary lubrication valve)
#   IN: [duration_H, duration_L] | OUT: [0x03, 0x01, ...]
# 0x04: Activate Valve 2 (Secondary lubrication valve)
#   IN: [duration_H, duration_L] | OUT: [0x04, 0x01, ...]
# 0x05: Move Feeder (Speed mode with direction & acceleration)
#   IN: [speed_H, speed_L, direction, acceleration] | OUT: [0x05, status, ...]
# 0x06: Read Pre-feeder Counter
#   IN: None | OUT: [0x06, 0x01, counter_H, counter_L, ...]
# 0x07: Read Feeder Counter
#   IN: None | OUT: [0x07, 0x01, counter_H, counter_L, ...]
# 0x08: Reset Feeder Counter
#   IN: None | OUT: [0x08, 0x01, ...]
# 0x09: Reset Pre-feeder Counter
#   IN: None | OUT: [0x09, 0x01, ...]
# 0x0A: Read Valve 1 Counter
#   IN: None | OUT: [0x0A, 0x01, counter_H, counter_L, ...]
# 0x0B: Read Valve 2 Counter
#   IN: None | OUT: [0x0B, 0x01, counter_H, counter_L, ...]
# 0x0C: Reset Valve 1 Counter
#   IN: None | OUT: [0x0C, 0x01, ...]
# 0x0D: Reset Valve 2 Counter
#   IN: None | OUT: [0x0D, 0x01, ...]
# 0x10: Read Feeder Counter (duplicate of 0x07)
#   IN: None | OUT: [0x10, 0x01, counter_H, counter_L, ...]
# 0x11: Read Pre-feeder Counter (duplicate of 0x06)
#   IN: None | OUT: [0x11, 0x01, counter_H, counter_L, ...]
# 0x12: Reset Feeder Counter (duplicate of 0x08)
#   IN: None | OUT: [0x12, 0x01, ...]
# 0x13: Move Pre-feeder (Speed mode with direction & acceleration)
#   IN: [speed_H, speed_L, direction, acceleration] | OUT: [0x13, status, ...]
# 0x14: Conditional Dual Movement (Feeder + conditional pre-feeder based on potentiometer)
#   IN: [speed_H, speed_L, direction, acceleration] | OUT: [0x14, status, ...]
# 0x15: Check Hose Position (IR break beam sensor status)
#   IN: None | OUT: [0x15, 0x01, sensor_status, ...]
# 0x16: Check Alcohol Level (Tank level using 2 digital sensors)
#   IN: None | OUT: [0x16, 0x01, level_status, ...]
# 0x17: Open Hose Holder (Servo to 90 degrees)
#   IN: None | OUT: [0x17, 0x01, ...]
# 0x18: Close Hose Holder (Servo to 0 degrees)
#   IN: None | OUT: [0x18, 0x01, ...]
# 0x19: Attach Electromagnet (Activate electromagnet)
#   IN: None | OUT: [0x19, 0x01, ...]
# 0x1A: Detach Electromagnet (Deactivate electromagnet)
#   IN: None | OUT: [0x1A, 0x01, ...]
# 0x1B: Read Servo Counter (Hose holder servo usage counter)
#   IN: None | OUT: [0x1B, 0x01, counter_H, counter_L, ...]
# 0x1C: Reset Servo Counter (Clear servo usage counter)
#   IN: None | OUT: [0x1C, 0x01, ...]
# 0x1D: Acknowledge Message (Send acknowledgment response)
#   IN: None | OUT: [0x1D, 0x01, ...]
# 0xFF: Emergency Stop (Stop all actuators immediately)
#   IN: None | OUT: None (stops all actuators)

class LubricationFeeder:
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

    # 0x03: Activate Valve 1 (Primary lubrication valve)
    def activate_valve_1(self, duration=1000):
        duration_high = (duration >> 8) & 0xFF
        duration_low = duration & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x03, duration_high, duration_low] + [0x00]*5)[0]
        return status

    # 0x04: Activate Valve 2 (Secondary lubrication valve)
    def activate_valve_2(self, duration=1000):
        duration_high = (duration >> 8) & 0xFF
        duration_low = duration & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x04, duration_high, duration_low] + [0x00]*5)[0]
        return status

    # 0x05: Move Feeder (Speed mode with direction & acceleration)
    def move_feeder(self, speed, direction=1, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x05, speed_high, speed_low, direction, acceleration] + [0x00]*3)[0]
        return status

    # 0x06: Read Pre-feeder Counter
    def get_pre_feeder_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x07: Read Feeder Counter
    def get_feeder_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x08: Reset Feeder Counter
    def reset_feeder_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x08] + [0x00]*7)[0]
        return status

    # 0x09: Reset Pre-feeder Counter
    def reset_pre_feeder_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x09] + [0x00]*7)[0]
        return status

    # 0x0A: Read Valve 1 Counter
    def get_valve_1_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x0B: Read Valve 2 Counter
    def get_valve_2_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x0C: Reset Valve 1 Counter
    def reset_valve_1_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x0C] + [0x00]*7)[0]
        return status

    # 0x0D: Reset Valve 2 Counter
    def reset_valve_2_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x0D] + [0x00]*7)[0]
        return status

    # 0x13: Move Pre-feeder (Speed mode with direction & acceleration)
    def move_pre_feeder(self, speed, direction=1, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x13, speed_high, speed_low, direction, acceleration] + [0x00]*3)[0]
        return status

    # 0x14: Conditional Dual Movement (Feeder + conditional pre-feeder based on potentiometer)
    def conditional_dual_movement(self, speed, direction=1, acceleration=236):
        speed_high = (speed >> 8) & 0xFF
        speed_low = speed & 0xFF
        status = self.canbus.send_message(self.canbus_id, [0x14, speed_high, speed_low, direction, acceleration] + [0x00]*3)[0]
        return status

    # 0x15: Check Hose Position (IR break beam sensor status)
    def check_hose_position(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x15] + [0x00]*7)
        if status == 1 and len(reply_data) >= 3:
            sensor_status = reply_data[2]
            return status, sensor_status
        return status, 0

    # 0x16: Check Alcohol Level (Tank level using 2 digital sensors)
    def check_alcohol_level(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x16] + [0x00]*7)
        if status == 1 and len(reply_data) >= 3:
            level_status = reply_data[2]  # 0=EMPTY, 1=MEDIUM, 2=FULL
            return status, level_status
        return status, 0

    # 0x17: Open Hose Holder (Servo to 90 degrees)
    def open_hose_holder(self):
        status = self.canbus.send_message(self.canbus_id, [0x17] + [0x00]*7)[0]
        return status

    # 0x18: Close Hose Holder (Servo to 0 degrees)
    def close_hose_holder(self):
        status = self.canbus.send_message(self.canbus_id, [0x18] + [0x00]*7)[0]
        return status

    # 0x19: Attach Electromagnet (Activate electromagnet)
    def attach_electromagnet(self):
        status = self.canbus.send_message(self.canbus_id, [0x19] + [0x00]*7)[0]
        return status

    # 0x1A: Detach Electromagnet (Deactivate electromagnet)
    def detach_electromagnet(self):
        status = self.canbus.send_message(self.canbus_id, [0x1A] + [0x00]*7)[0]
        return status

    # 0x1B: Read Servo Counter (Hose holder servo usage counter)
    def get_servo_counter(self):
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x1B] + [0x00]*7)
        if status == 1 and len(reply_data) >= 4:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0

    # 0x1C: Reset Servo Counter (Clear servo usage counter)
    def reset_servo_counter(self):
        status = self.canbus.send_message(self.canbus_id, [0x1C] + [0x00]*7)[0]
        return status

    # 0x1D: Acknowledge Message (Send acknowledgment response)
    def acknowledge_message(self):
        status = self.canbus.send_message(self.canbus_id, [0x1D] + [0x00]*7)[0]
        return status

    # 0xFF: Emergency Stop (Stop all actuators immediately)
    def emergency_stop(self):
        status = self.canbus.send_message(self.canbus_id, [0xFF] + [0x00]*7)[0]
        return status

    # Convenience methods for common operations
    def lubricate_nozzle(self, duration=1000):
        """Activate primary lubrication valve for specified duration"""
        return self.activate_valve_1(duration)

    def lubricate_joint(self, duration=1000):
        """Activate secondary lubrication valve for specified duration"""
        return self.activate_valve_2(duration)

    def feed_hose(self, speed=1000, direction=1, duration=3):
        """Move main feeder at specified speed and direction for a duration, then stop"""
        import time
        
        # Start movement with specified speed
        status = self.move_feeder(speed, direction)
        if status != 1:
            return status
        
        # Wait for specified duration
        time.sleep(duration)
        
        # Stop movement by sending 0 speed
        stop_status = self.move_feeder(0, direction)
        return stop_status

    def pre_feed_hose(self, speed=1000, direction=1, duration=3):
        """Move pre-feeder at specified speed and direction for a duration, then stop"""
        import time
        
        # Start movement with specified speed
        status = self.move_pre_feeder(speed, direction)
        if status != 1:
            return status
        
        # Wait for specified duration
        time.sleep(duration)
        
        # Stop movement by sending 0 speed
        stop_status = self.move_pre_feeder(0, direction)
        return stop_status

    def is_hose_present(self):
        """Check if hose is present using IR sensor"""
        status, sensor_status = self.check_hose_position()
        return status == 1 and sensor_status == 1

    def get_tank_level(self):
        """Get alcohol tank level (0=EMPTY, 1=MEDIUM, 2=FULL)"""
        status, level = self.check_alcohol_level()
        if status == 1:
            return level
        return -1  # Error

    def hold_hose(self):
        """Open hose holder to grip hose"""
        return self.open_hose_holder()

    def release_hose(self):
        """Close hose holder to release hose"""
        return self.close_hose_holder()

    def engage_magnet(self):
        """Activate electromagnet"""
        return self.attach_electromagnet()

    def disengage_magnet(self):
        """Deactivate electromagnet"""
        return self.detach_electromagnet()