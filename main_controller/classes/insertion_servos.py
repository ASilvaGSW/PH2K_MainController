
import time

class InsertionServos:
    
    # Slider Joint (S3) 0 - 160 , origen 160 - 0 insercion
    # Clamp Joint (S4) 20 - 108 , abierto 20 - cerrado 108
    # Holder hose (Joint side) (S7) 0 - 121, 0 abierto - 121 cerrado
    # Slider nozzle (S5) 125 - 0 , 125 origen - 0 insercion
    # Clamp nozzle (S6) 47 - 140,  47 cerrado - 140 abierto
    # Holder Hose (Nozzle side) (S8) 0 - 122, abierto 0 - 122 cerrado
    # Cutter (S9) 0 - 80, abierto 0 - 80 cerrado

    def __init__(self, canbus, canbus_id):
        self.canbus = canbus
        self.canbus_id = canbus_id

        self.slider_joint_home_pos = 160
        self.slider_joint_receive_pos = 150
        self.slider_joint_insertion_pos = 8

        self.clamp_joint_open_pos = 20
        self.clamp_joint_close_pos = 108

        #works for both sides joint and nozzle
        self.holder_hose_open_pos = 0
        self.holder_hose_close_pos = 115
        self.holder_hose_close_pos2 = 125

        self.slider_nozzle_home_pos = 125
        self.slider_nozzle_receive_pos = 125
        self.slider_nozzle_insertion_pos = 0

        self.clamp_nozzle_open_pos = 110
        self.clamp_nozzle_close_pos = 41

        self.cutter_open_pos = 0
        self.cutter_close_pos = 80


    def reset_micro(self):
        """Reset the microcontroller"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    def send_heartbeat(self):
        """Send a heartbeat command to check if the device is alive"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],max_retries=30)
        return status

    def slider_joint(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the slider joint"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x03, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    def clamp_joint(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the clamp joint"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x04, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    def holder_hose_joint(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the holder hose joint"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x05, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    def slider_nozzle(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the slider nozzle"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x06, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    def clamp_nozzle(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the clamp nozzle"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x07, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    def holder_hose_nozzle(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the holder hose nozzle"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x08, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status
    
    def cutter(self,angle):
        angle_high = (angle >> 8) & 0xFF
        angle_low = angle & 0xFF
        """Move the cutter"""
        status, reply_data = self.canbus.send_message(self.canbus_id,[0x09, angle_high, angle_low, 0x00, 0x00, 0x00, 0x00, 0x00])
        return status

    #Custom Functions

    def activate_cutter(self):
        self.cutter(80)
        time.sleep(0.5)
        return self.cutter(1)

    def slider_joint_insertion(self, position=None):
        if position is None:
            position = self.slider_joint_insertion_pos
        return self.slider_joint(position)
    
    def slider_joint_home(self):
        return self.slider_joint(self.slider_joint_home_pos)

    def slider_joint_receive(self):
        return self.slider_joint(self.slider_joint_receive_pos)

    def slider_nozzle_receive(self):
        return self.slider_nozzle(self.slider_nozzle_receive_pos)

    def clamp_joint_open(self):
        return self.clamp_joint(self.clamp_joint_open_pos)
    
    def clamp_joint_close(self):
        return self.clamp_joint(self.clamp_joint_close_pos)

    def holder_hose_nozzle_open(self):
        return self.holder_hose_nozzle(self.holder_hose_open_pos)
    
    def holder_hose_nozzle_close(self):
        return self.holder_hose_nozzle(self.holder_hose_close_pos2)

    def holder_hose_joint_open(self):
        return self.holder_hose_joint(self.holder_hose_open_pos)
    
    def holder_hose_joint_close(self):
        return self.holder_hose_joint(self.holder_hose_close_pos)

    def holder_hose_joint_semi_close(self):
        return self.holder_hose_joint(self.holder_hose_close_pos-11)

    def slider_nozzle_insertion(self, position=None):
        if position is None:
            position = self.slider_nozzle_insertion_pos
        return self.slider_nozzle(position)
    
    def slider_nozzle_home(self):
        return self.slider_nozzle(self.slider_nozzle_home_pos)
    
    def clamp_nozzle_open(self):
        return self.clamp_nozzle(self.clamp_nozzle_open_pos)
    
    def clamp_nozzle_close(self):
        return self.clamp_nozzle(self.clamp_nozzle_close_pos)

    def cutter_open(self):
        return self.cutter(self.cutter_open_pos)
    
    def cutter_close(self):
        return self.cutter(self.cutter_close_pos)




