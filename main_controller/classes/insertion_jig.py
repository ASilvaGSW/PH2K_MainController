"""
Insertion Jig Control System 0x0C9

This module controls a multi-axis insertion jig system using CAN bus communication.
It supports both TWAI (ESP32 Integrated CAN) and MCP2515-based CAN interfaces.
"""

class InsertionJig:
    """
    A class to control the Insertion Jig system via CAN bus.
    
    This class provides methods to interact with the Insertion Jig hardware,
    including controlling the X and Z axes, reading movement counters, and
    managing predefined positions.
    
    Attributes:
        canbus: The CAN bus interface object.
        canbus_id: The CAN bus ID for the Insertion Jig device.
    """
    
    def __init__(self, canbus, canbus_id):
        """
        Initialize the InsertionJig with CAN bus interface and ID.
        
        Args:
            canbus: The CAN bus interface object.
            canbus_id: The CAN bus ID for the Insertion Jig device.
        """
        self.canbus = canbus
        self.canbus_id = canbus_id
    
    # Case 0x01: Reset Microcontroller
    def reset_microcontroller(self):
        """
        Reset the microcontroller.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x02: Heartbeat
    def send_heartbeat(self):
        """
        Send a heartbeat message to verify the device is responsive.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x03: Home X Axis
    def home_x_axis(self):
        """
        Move the X-axis actuator to its home position (0).
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x04: Move X Axis to Absolute Position
    def move_x_axis(self, position,flag = True):
        """
        Move the X-axis to an absolute position.
        
        Args:
            position (int): The target position for the X-axis.
            
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        if position < 0:
            orientation = 1
        else:
            orientation = 0
            
        position = abs(position)
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        
        status, reply_data = self.canbus.send_message(
            self.canbus_id, 
            [0x04, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00],flag
        )
        return status
    
    # Case 0x05: Home Z Axis
    def home_z_axis(self):
        """
        Move all Z-axis actuators to their home positions (0).
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x06: Move Z Axis to Absolute Position
    def move_z_axis(self, position,flag = True):
        """
        Move all Z-axis actuators to an absolute position.
        
        Args:
            position (int): The target position for the Z-axis.
            
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        if position < 0:
            orientation = 1
        else:
            orientation = 0
            
        position = abs(position)
        position_high = (position >> 8) & 0xFF
        position_low = position & 0xFF
        
        status, reply_data = self.canbus.send_message(
            self.canbus_id, 
            [0x06, position_high, position_low, orientation, 0x00, 0x00, 0x00, 0x00],flag
        )
        return status
    
    # Case 0x08: Get X Movement Counter
    def x_counter(self):
        """
        Get the number of movements made by the X-axis actuator.
        
        Returns:
            tuple: (status, counter) where counter is the movement count.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0
    
    # Case 0x09: Reset X Movement Counter
    def reset_x_counter(self):
        """
        Reset the X-axis movement counter to zero.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x0A: Get Z Movement Counter
    def z_counter(self):
        """
        Get the number of movements made by the Z-axis actuators.
        
        Returns:
            tuple: (status, counter) where counter is the movement count.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if status:
            counter = (reply_data[2] << 8) | reply_data[3]
            return status, counter
        return status, 0
    
    # Case 0x0B: Reset Z Movement Counter
    def reset_z_counter(self):
        """
        Reset the Z-axis movement counter to zero.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x0C: Move to Predefined Position
    def move_to_predefined_position(self, position_id):
        """
        Move the X-axis to a predefined position based on position ID.
        
        Args:
            position_id (int): The ID of the predefined position.
            
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(
            self.canbus_id, 
            [0x0C, position_id & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        return status
    
    # Case 0x0D: Home X Axis using go_home function
    def home_x_axis_go_home(self):
        """
        Move the X-axis actuator to its home position using the go_home function.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    # Case 0x0E: Home Z Axis using go_home function
    def home_z_axis_go_home(self):
        """
        Move all Z-axis actuators to their home positions using the go_home function.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    def x_lubricate_nozzle(self):

        return self.move_to_predefined_position(1)
    
    def x_insertion_nozzle(self):
        return self.move_to_predefined_position(2)
    
    def x_lubricate_joint(self):
        return self.move_to_predefined_position(3)
    
    def x_insertion_joint(self):
        return self.move_to_predefined_position(4)

    
        
    
    # Case 0xFF: Power Off
    def power_off(self):
        """
        Move all axes to home position and prepare for power off.
        
        Returns:
            tuple: (status, reply_data) from the CAN bus.
        """
        status, reply_data = self.canbus.send_message(self.canbus_id, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status