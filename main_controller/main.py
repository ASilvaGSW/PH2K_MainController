import time
import platform
import sys

# Detectar sistema operativo para usar la clase Canbus adecuada
if platform.system() == 'Linux':
    # En Jetson Orin Nano (Linux) usamos la implementación con python-can
    try:
        from classes.canbus_jetson import Canbus
        print("Usando implementación de Canbus para Jetson Orin Nano")
    except ImportError as e:
        print(f"Error al importar canbus_jetson: {e}")
        sys.exit(1)
else:
    # En Windows usamos la implementación original con candle_driver
    try:
        from classes.canbus import Canbus
        print("Usando implementación de Canbus para Windows")
    except ImportError as e:
        print(f"Error al importar canbus: {e}")
        sys.exit(1)

from classes.hose_jig import HoseJig
from classes.hose_puller import HosePuller
from classes.puller_extension import PullerExtension
from classes.insertion_jig import InsertionJig
from classes.elevator_in import ElevatorIn
from classes.pick_and_place import PickAndPlace
from classes.insertion_servos import InsertionServos

# Variables globales para acceder a los objetos desde cualquier función
canbus = None
hose_jig = None
hose_puller = None
puller_extension = None
insertion_jig = None
elevator_in = None
pick_and_place = None
insertion_servos = None

def my_main():
    global canbus, hose_jig, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos
    
    # Example CAN IDs (these should be configured according to your hardware setup)
    CANBUS_ID_JIG = 0x0CA
    CANBUS_ID_PULLER = 0x192
    CANBUS_ID_EXTENSION = 0x193
    CANBUS_ID_INSERTION = 0x0C9
    CANBUS_ID_ELEVATOR_IN = 0x189   
    CANBUS_ID_PICK_AND_PLACE = 0x191
    CANBUS_ID_INSERTION_SERVOS = 0x002

    # Setup CANbus and devices
    canbus = Canbus()
    canbus.start_canbus()
    
    hose_jig = HoseJig(canbus, CANBUS_ID_JIG)
    hose_puller = HosePuller(canbus, CANBUS_ID_PULLER)
    puller_extension = PullerExtension(canbus, CANBUS_ID_EXTENSION)
    insertion_jig = InsertionJig(canbus, CANBUS_ID_INSERTION)
    elevator_in = ElevatorIn(canbus, CANBUS_ID_ELEVATOR_IN)
    pick_and_place = PickAndPlace(canbus, CANBUS_ID_PICK_AND_PLACE)
    insertion_servos = InsertionServos(canbus, CANBUS_ID_INSERTION_SERVOS)

    # for i in range(2):
    #     hose_jig.send_heartbeat()
    #     hose_puller.send_heartbeat()
    #     puller_extension.send_heartbeat()
    #     pick_and_place.send_heartbeat()
    #     insertion_jig.send_heartbeat()
    #     elevator_in.send_heartbeat()
    #     insertion_servos.send_heartbeat()

def moveHosepuller():
    global hose_puller, hose_jig, puller_extension,insertion_servos, insertion_jig

    safe_position = 200
    home_y = 4200
    wait_y = 6000
    pickup_y = 9060
    z_home = 50

    offset_x = -480
    offset_z = -20

    # insertion_jig.move_z_axis(0)
    # insertion_jig.move_x_axis(0)

    #****************************** Hose puller ******************************
    # hose_puller.move_y_actuator(0)
    # hose_puller.move_z_actuator(0)

    # hose_puller.move_y_actuator(home_y)
    # hose_puller.move_z_actuator(safe_position)
    # hose_jig.insertion_position()
    # hose_puller.move_y_actuator(pickup_y)
    # hose_puller.move_z_actuator(z_home)
    # puller_extension.close_gripper()
    # hose_puller.move_y_actuator(pickup_y-500)
    # hose_puller.move_z_actuator(safe_position)
    # hose_puller.move_y_actuator(wait_y)
    # insertion_servos.activate_cutter()
    # hose_puller.move_y_actuator(home_y)
    # hose_puller.move_z_actuator(z_home)
    # hose_jig.gripper_close()
    # puller_extension.open_gripper()
    # hose_puller.move_z_actuator(safe_position)
    # hose_jig.deliver_position()
    # hose_jig.gripper_open()
    # hose_puller.move_z_actuator(0)
    
def moveInsertionJig():
    global insertion_jig, insertion_servos


     #****************************** Insertion Jig ******************************

    offset_x = -480
    offset_z = -20

    home_position_z = 3000 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = -2310 + offset_z
    insertion_position_z = -2360 + offset_z
    insertion_position_joint_z = -2240 + offset_z
    librication_position_joint_z = -2250 + offset_z

    lubricate_nozzle = -5530 + offset_x
    insert_nozzle = -6790 + offset_x  
    insert_joint = -9110 + offset_x
    lubricate_joint = -11200 + offset_x

    # XZ positions

    # insertion_jig.move_z_axis(home_position_z)
    # insertion_jig.move_x_axis(home_position_x)

    # insertion_jig.move_z_axis(lubrication_position_z)
    # insertion_jig.move_x_axis(lubricate_nozzle)

    # insertion_jig.move_z_axis(insertion_position_z)
    # insertion_jig.move_x_axis(insert_nozzle)

    # insertion_jig.move_z_axis(librication_position_joint_z)
    # insertion_jig.move_x_axis(lubricate_joint)

    # insertion_jig.move_z_axis(insertion_position_joint_z)
    # insertion_jig.move_x_axis(insert_joint)

    # insertion_jig.move_z_axis(0)
    # insertion_jig.move_x_axis(0)
   
    
    #Nozzle Insertion

    # insertion_servos.holder_hose_nozzle_close()
    # insertion_servos.clamp_nozzle_close()
    # time.sleep(.5)
    # insertion_servos.slider_nozzle_insertion()
    # time.sleep(1.0)
    # insertion_servos.holder_hose_nozzle_open()
    # insertion_servos.clamp_nozzle_open()
    # time.sleep(.5)
    # insertion_servos.slider_nozzle_home()
    # insertion_servos.activate_cutter()
    

    #Joint Insertion

    # insertion_servos.holder_hose_joint_close()
    # insertion_servos.clamp_joint_close()
    # time.sleep(0.5)
    # insertion_servos.slider_joint_insertion()
    # time.sleep(1)
    # insertion_servos.holder_hose_joint_open()
    # insertion_servos.clamp_joint_open()
    # time.sleep(0.5)
    # insertion_servos.slider_joint_home()

def insertionRoutine():
    global insertion_jig, insertion_servos

    # insertion_servos.slider_joint_home()

    #****************************** Insertion Jig ******************************

    offset_x = -310
    offset_z = -20

    home_position_z = 3000 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = -2310 + offset_z
    insertion_position_z = -2360 + offset_z
    insertion_position_joint_z = -2230 + offset_z
    librication_position_joint_z = -2250 + offset_z

    lubricate_nozzle = -5530 + offset_x
    insert_nozzle = -6790 + offset_x  
    insert_joint = -9190 + offset_x
    lubricate_joint = -11200 + offset_x

    #****************************** Routine ******************************

    #Nozzle Position

    
    # insertion_jig.move_z_axis(home_position_z)
    # insertion_jig.move_x_axis(home_position_x)

    # insertion_jig.move_z_axis(lubrication_position_z)
    # insertion_jig.move_x_axis(lubricate_nozzle)

    # insertion_jig.move_z_axis(insertion_position_z)
    # insertion_jig.move_x_axis(insert_nozzle)
  
    
    #Nozzle Insertion


    # insertion_servos.holder_hose_nozzle_close()
    # insertion_servos.clamp_nozzle_close()
    # time.sleep(.5)
    # insertion_servos.slider_nozzle_insertion()
    # time.sleep(1)
    # insertion_servos.holder_hose_nozzle_open()
    # insertion_servos.clamp_nozzle_open()
    # time.sleep(.5)
    # insertion_servos.slider_nozzle_home()

    # insertion_servos.activate_cutter()

    # #Moving to Joint

    # insertion_jig.move_z_axis(librication_position_joint_z)
    # insertion_jig.move_x_axis(lubricate_joint)

    # insertion_jig.move_z_axis(insertion_position_joint_z)
    # insertion_jig.move_x_axis(insert_joint)
   

    # #Clamp Insertion

    # insertion_servos.holder_hose_joint_close()
    # insertion_servos.clamp_joint_close()
    # time.sleep(0.5)
    # insertion_servos.slider_joint_insertion()
    # time.sleep(1)
    # insertion_servos.holder_hose_joint_open()
    # insertion_servos.clamp_joint_open()
    # time.sleep(0.5)
    # insertion_servos.slider_joint_home()


    # insertion_jig.move_z_axis(0)
    # insertion_jig.move_x_axis(0)

def moveElevatorIn():
    global elevator_in,pick_and_place

    home = [0,0,0]
    safety_high = -300
    cassette_deliver_high = -630
    transportation_high = -500

    #left

    pick_left = [-252,-330,-775]
    deliver_left = [-260,-1800,-500]
    
    #Right

    pick_right = [-1380,-330,-775]
    deliver_right = [-1380,-1800,-500]

    #Alignin Elevator

    # elevator_in.move_elevator_z(-4200)
    
    # return ""

    #Left Side
    
    elevator_in.move_gantry_array(pick_left)

    elevator_in.close_gripper()

    elevator_in.move_gantry_z(transportation_high)

    elevator_in.move_gantry_array(deliver_left)

    elevator_in.move_gantry_z(cassette_deliver_high)

    elevator_in.open_gripper()

    elevator_in.move_gantry_z(safety_high)

    elevator_in.move_gantry_array(home)

    # #Right Side

    elevator_in.move_gantry_array(pick_right)

    elevator_in.close_gripper()

    elevator_in.move_gantry_z(transportation_high)

    elevator_in.move_gantry_array(deliver_right)

    elevator_in.move_gantry_z(cassette_deliver_high)

    elevator_in.open_gripper()

    elevator_in.move_gantry_z(safety_high)

    elevator_in.move_gantry_array(home)

    # #conveyors

    pick_and_place.start_left_conveyor()
    pick_and_place.start_right_conveyor()

    time.sleep(5)

    pick_and_place.stop_left_conveyor()
    pick_and_place.stop_right_conveyor()

def movePickandPlace():
    global pick_and_place, insertion_servos, insertion_jig

    # Homing Insertion Jig
    insertion_servos.slider_joint_home()
    insertion_jig.move_z_axis(5000)
    insertion_jig.move_x_axis(-8500)


    #Home Pick and Place
    pick_and_place.move_actuator_z(0)
    pick_and_place.move_actuator_x(0)
    pick_and_place.open_gripper()

    # #Pick Nozzle
    pick_and_place.move_actuator_x(-1800)
    pick_and_place.move_actuator_z(-1250)
    pick_and_place.close_gripper()

    # #Safe Position 1
    pick_and_place.move_actuator_z(-1000)

    # #Place Nozzle
    pick_and_place.move_actuator_x(-4090)
    pick_and_place.move_actuator_z(-1380)
    pick_and_place.open_gripper()

    #Safe Position 2
    pick_and_place.move_actuator_z(-1000)

    # #Pick Joint
    pick_and_place.move_actuator_x(-850)
    pick_and_place.move_actuator_z(-1250)
    pick_and_place.close_gripper()

    #Safe Position 3
    pick_and_place.move_actuator_z(-1000)

    # Place Joint
    pick_and_place.move_actuator_x(-4390)
    pick_and_place.move_actuator_z(-1320)
    pick_and_place.open_gripper()

    pick_and_place.move_actuator_z(0)
    pick_and_place.move_actuator_x(0)
    pick_and_place.open_gripper()

def oneCycle():

    global insertion_jig, insertion_servos, hose_puller, hose_jig, puller_extension,pick_and_place

    #****************************** Insertion Jig ******************************

    offset_x = -310
    offset_z = -20

    home_position_z = 3000 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = -2310 + offset_z
    insertion_position_z = -2360 + offset_z
    insertion_position_joint_z = -2230 + offset_z
    librication_position_joint_z = -2250 + offset_z

    lubricate_nozzle = -5530 + offset_x
    insert_nozzle = -6790 + offset_x  
    insert_joint = -9190 + offset_x
    lubricate_joint = -11200 + offset_x

    safe_position = 200
    home_y = 4200
    wait_y = 6000
    pickup_y = 9060
    z_home = 50


    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"


    #****************************** Routine ******************************

    #Nozzle Position

    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error02"

    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error04"

    if insertion_jig.move_z_axis(insertion_position_z) != "success" : return "error05"
    if insertion_jig.move_x_axis(insert_nozzle) != "success" : return "error06"
  
    
    # Nozzle Insertion


    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success" : return "error09"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success" : return "error10"
    if insertion_servos.clamp_nozzle_open() != "success" : return "error11"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_home() != "success" : return "error12"


    #Go to Down Position for Hose Puller

    if insertion_jig.move_z_axis(1000) != "success" : return "error13"

    #Hose Puller Action

    if hose_puller.move_y_actuator(home_y) != "success" : return "error03"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error04"
    if hose_jig.insertion_position() != "success" : return "error05"
    if hose_puller.move_y_actuator(pickup_y) != "success" : return "error06"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error07"
    if puller_extension.close_gripper() != "success" : return "error08"
    if hose_puller.move_y_actuator(pickup_y-500) != "success" : return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error10"
    if hose_puller.move_y_actuator(wait_y) != "success" : return "error11"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success" : return "error12"
    if hose_puller.move_y_actuator(home_y+720) != "success" : return "error13"
    
    #Moving to Joint

    if insertion_servos.holder_hose_joint_close() != "success" : return "error18"

    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error15"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error14"

    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error17"
    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success" : return "error16"
   

    #Clamp Insertion

   
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success" : return "error20"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success" : return "error21"
    if insertion_servos.clamp_joint_open() != "success" : return "error22"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success" : return "error23"


    #Finish Pulling Action

    if hose_puller.move_y_actuator(home_y) != "success" : return "error13"

    if insertion_jig.move_z_axis(0) != "success" : return "error24"
    if insertion_jig.move_x_axis(0) != "success" : return "error25"


    if hose_puller.move_z_actuator(z_home) != "success" : return "error14"
    if hose_jig.gripper_close() != "success" : return "error15"
    if puller_extension.open_gripper() != "success" : return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error17"
    if hose_jig.deliver_position() != "success" : return "error18"
    if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success" : return "error20"

    return "success"

if __name__ == "__main__":
   
    my_main()

    # moveElevatorIn()

    # movePickandPlace()

    # insertionRoutine()

    # moveHosepuller()

    print(oneCycle())

    canbus.close_canbus()


