# LUBRICATION STAMPER 0x004
# CAN Command Cases (input/output summary):
# 0x01: Reset microcontroller
#   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
# 0x02: Heartbeat
#   IN: None | OUT: [0x02, 0x01, ...]
# 0x03: Read individual level sensor (data[1] = sensor number 1-3)
#   IN: [sensor_number] | OUT: [0x03, 0x01, sensor_number, level_status, ...]
# 0x04: Read all level sensors (returns status of all 3 sensors)
#   IN: None | OUT: [0x04, 0x01, tank1_status, tank2_status, tank3_status, ...]
# 0x05: Read flow meter rate (data[1] = meter number 1-3, returns float L/min)
#   IN: [meter_number] | OUT: [0x05, 0x01, meter_number, rate_bytes[4], ...]
# 0x06: Read flow meter total pulses (data[1] = meter number 1-3, returns unsigned long)
#   IN: [meter_number] | OUT: [0x06, 0x01, meter_number, pulses_bytes[4], ...]
# 0x07: Reset flow meter (data[1] = meter number 1-3)
#   IN: [meter_number] | OUT: [0x07, 0x01, meter_number, ...]
# 0x08: Read all flow meter rates (returns all 3 rates as 16-bit integers * 100)
#   IN: None | OUT: [0x08, 0x01, rate1_H, rate1_L, rate2_H, rate2_L, rate3_H, rate3_L]
# 0x09: Set solenoid valve state (data[1] = solenoid number 1-3, data[2] = state 0/1)
#   IN: [solenoid_number, state] | OUT: [0x09, 0x01, solenoid_number, state, ...]
# 0x0A: Get solenoid valve state (data[1] = solenoid number 1-3)
#   IN: [solenoid_number] | OUT: [0x0A, 0x01, solenoid_number, state, ...]
# 0x0B: Toggle solenoid valve (data[1] = solenoid number 1-3)
#   IN: [solenoid_number] | OUT: [0x0B, 0x01, solenoid_number, new_state, ...]
# 0x0C: Get all solenoid states (returns status of all 3 solenoids)
#   IN: None | OUT: [0x0C, 0x01, solenoid1_state, solenoid2_state, solenoid3_state, ...]
# 0x0D: Emergency close all solenoids (safety command)
#   IN: None | OUT: [0x0D, 0x01, ...]
# 0xFF: Power off command
#   IN: None | OUT: [0xFF, 0x01, ...]

import struct

class LubricationStamper:
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

    # Case 0x03: Read individual level sensor
    def read_level_sensor(self, sensor_number):
        """
        Read individual level sensor status
        Args:
            sensor_number (int): Sensor number (1-3)
        Returns:
            tuple: (status, level_detected) where level_detected is True if liquid detected
        """
        if sensor_number < 1 or sensor_number > 3:
            return False, False
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03, sensor_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            level_detected = reply_data[3] == 0x01
            return status, level_detected
        return status, False

    # Case 0x04: Read all level sensors
    def read_all_level_sensors(self):
        """
        Read all level sensors status
        Returns:
            tuple: (status, [tank1_level, tank2_level, tank3_level])
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 5:
            levels = [
                reply_data[2] == 0x01,  # Tank 1
                reply_data[3] == 0x01,  # Tank 2
                reply_data[4] == 0x01   # Tank 3
            ]
            return status, levels
        return status, [False, False, False]

    # Case 0x05: Read flow meter rate
    def read_flow_meter_rate(self, meter_number):
        """
        Read flow meter rate in L/min
        Args:
            meter_number (int): Flow meter number (1-3)
        Returns:
            tuple: (status, flow_rate) where flow_rate is in L/min
        """
        if meter_number < 1 or meter_number > 3:
            return False, 0.0
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, meter_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 7:
            # Convert bytes back to float (IEEE 754 format)
            flow_bytes = bytes(reply_data[3:7])
            flow_rate = struct.unpack('<f', flow_bytes)[0]
            return status, flow_rate
        return status, 0.0

    # Case 0x06: Read flow meter total pulses
    def read_flow_meter_pulses(self, meter_number):
        """
        Read flow meter total pulses
        Args:
            meter_number (int): Flow meter number (1-3)
        Returns:
            tuple: (status, total_pulses)
        """
        if meter_number < 1 or meter_number > 3:
            return False, 0
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x06, meter_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 7:
            # Convert bytes back to unsigned long
            total_pulses = (reply_data[3] << 24) | (reply_data[4] << 16) | (reply_data[5] << 8) | reply_data[6]
            return status, total_pulses
        return status, 0

    # Case 0x07: Reset flow meter
    def reset_flow_meter(self, meter_number):
        """
        Reset flow meter pulse counter
        Args:
            meter_number (int): Flow meter number (1-3)
        Returns:
            bool: Success status
        """
        if meter_number < 1 or meter_number > 3:
            return False
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x07, meter_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x08: Read all flow meter rates
    def read_all_flow_meter_rates(self):
        """
        Read all flow meter rates
        Returns:
            tuple: (status, [rate1, rate2, rate3]) where rates are in L/min
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 8:
            # Unpack 16-bit integers and convert back to float (divided by 100)
            rate1 = ((reply_data[2] << 8) | reply_data[3]) / 100.0
            rate2 = ((reply_data[4] << 8) | reply_data[5]) / 100.0
            rate3 = ((reply_data[6] << 8) | reply_data[7]) / 100.0
            return status, [rate1, rate2, rate3]
        return status, [0.0, 0.0, 0.0]

    # Case 0x09: Set solenoid valve state
    def set_solenoid_state(self, solenoid_number, state):
        """
        Set solenoid valve state
        Args:
            solenoid_number (int): Solenoid number (1-3)
            state (bool): True for open, False for closed
        Returns:
            bool: Success status
        """
        if solenoid_number < 1 or solenoid_number > 3:
            return False
        
        state_byte = 0x01 if state else 0x00
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09, solenoid_number, state_byte, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0x0A: Get solenoid valve state
    def get_solenoid_state(self, solenoid_number):
        """
        Get solenoid valve state
        Args:
            solenoid_number (int): Solenoid number (1-3)
        Returns:
            tuple: (status, is_open) where is_open is True if valve is open
        """
        if solenoid_number < 1 or solenoid_number > 3:
            return False, False
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A, solenoid_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            is_open = reply_data[3] == 0x01
            return status, is_open
        return status, False

    # Case 0x0B: Toggle solenoid valve
    def toggle_solenoid(self, solenoid_number):
        """
        Toggle solenoid valve state
        Args:
            solenoid_number (int): Solenoid number (1-3)
        Returns:
            tuple: (status, new_state) where new_state is True if valve is now open
        """
        if solenoid_number < 1 or solenoid_number > 3:
            return False, False
        
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, solenoid_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 4:
            new_state = reply_data[3] == 0x01
            return status, new_state
        return status, False

    # Case 0x0C: Get all solenoid states
    def get_all_solenoid_states(self):
        """
        Get all solenoid valve states
        Returns:
            tuple: (status, [solenoid1_state, solenoid2_state, solenoid3_state])
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status and len(reply_data) >= 5:
            states = [
                reply_data[2] == 0x01,  # Solenoid 1
                reply_data[3] == 0x01,  # Solenoid 2
                reply_data[4] == 0x01   # Solenoid 3
            ]
            return status, states
        return status, [False, False, False]

    # Case 0x0D: Emergency close all solenoids
    def emergency_close_all_solenoids(self):
        """
        Emergency close all solenoid valves (safety command)
        Returns:
            bool: Success status
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Case 0xFF: Power off
    def power_off(self):
        """Power off the system"""
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    # Convenience methods for specific operations
    def open_solenoid(self, solenoid_number):
        """Open a specific solenoid valve"""
        return self.set_solenoid_state(solenoid_number, True)

    def close_solenoid(self, solenoid_number):
        """Close a specific solenoid valve"""
        return self.set_solenoid_state(solenoid_number, False)

    def open_all_solenoids(self):
        """Open all solenoid valves"""
        results = []
        for i in range(1, 4):
            results.append(self.open_solenoid(i))
        return all(results)

    def close_all_solenoids(self):
        """Close all solenoid valves"""
        results = []
        for i in range(1, 4):
            results.append(self.close_solenoid(i))
        return all(results)

    def reset_all_flow_meters(self):
        """Reset all flow meter counters"""
        results = []
        for i in range(1, 4):
            results.append(self.reset_flow_meter(i))
        return all(results)

    def get_system_status(self):
        """
        Get comprehensive system status
        Returns:
            dict: Complete system status including levels, flows, and solenoid states
        """
        status_dict = {}
        
        # Get level sensors
        level_status, levels = self.read_all_level_sensors()
        status_dict['level_sensors'] = {
            'status': level_status,
            'tank1': levels[0] if level_status else False,
            'tank2': levels[1] if level_status else False,
            'tank3': levels[2] if level_status else False
        }
        
        # Get flow rates
        flow_status, rates = self.read_all_flow_meter_rates()
        status_dict['flow_meters'] = {
            'status': flow_status,
            'rate1': rates[0] if flow_status else 0.0,
            'rate2': rates[1] if flow_status else 0.0,
            'rate3': rates[2] if flow_status else 0.0
        }
        
        # Get solenoid states
        solenoid_status, states = self.get_all_solenoid_states()
        status_dict['solenoids'] = {
            'status': solenoid_status,
            'solenoid1': states[0] if solenoid_status else False,
            'solenoid2': states[1] if solenoid_status else False,
            'solenoid3': states[2] if solenoid_status else False
        }
        
        return status_dict