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
from classes.lubrication_feeder import LubricationFeeder

# Variables globales para acceder a los objetos desde cualquier función
canbus = None
hose_jig = None
hose_puller = None
puller_extension = None
insertion_jig = None
elevator_in = None
pick_and_place = None
insertion_servos = None
lubrication_feeder = None

def my_main():
    global canbus, hose_jig, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos, lubrication_feeder
    
    # Example CAN IDs (these should be configured according to your hardware setup)
    CANBUS_ID_JIG = 0x0CA
    CANBUS_ID_PULLER = 0x192
    CANBUS_ID_EXTENSION = 0x193
    CANBUS_ID_INSERTION = 0x0C9
    CANBUS_ID_ELEVATOR_IN = 0x189   
    CANBUS_ID_PICK_AND_PLACE = 0x191
    CANBUS_ID_INSERTION_SERVOS = 0x002
    CANBUS_ID_LUBRICATION_FEEDER = 0x019

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
    lubrication_feeder = LubricationFeeder(canbus, CANBUS_ID_LUBRICATION_FEEDER)

    if True:
        if hose_jig.send_heartbeat() != "success" : print("Hose Jig Not Connected")
        if hose_puller.send_heartbeat() != "success" : print("Hose Puller Not Connected")
        if puller_extension.send_heartbeat() != "success" : print("Puller Extension Not Connected")
        if pick_and_place.send_heartbeat() != "success" : print("Pick and Place Not Connected") 
        if insertion_jig.send_heartbeat() != "success" : print("Insertion Jig Not Connected")
        if elevator_in.send_heartbeat() != "success" : print("Elevator In Not Connected")
        if insertion_servos.send_heartbeat() != "success" : print("Insertion Servos Not Connected")
        if lubrication_feeder.send_heartbeat() != "success" : print("Lubrication Feeder Not Connected")

def moveHosepuller():
    global hose_puller, hose_jig, puller_extension,insertion_servos, insertion_jig

    safe_position = 200
    home_y = 4200
    wait_y = 5930
    pickup_y = 8990
    z_home = 50

    offset_x = -480
    offset_z = -20

    # insertion_jig.move_z_axis(0)
    # insertion_jig.move_x_axis(0)

    #****************************** Hose puller ******************************
    hose_puller.home_y_axis()
    hose_puller.home_z_axis()
    hose_jig.go_home()

    # hose_puller.move_y_actuator(0)
    # hose_puller.move_z_actuator(0)

    hose_puller.move_y_actuator(home_y)
    hose_puller.move_z_actuator(safe_position)
    hose_jig.insertion_position()
    hose_puller.move_y_actuator(pickup_y)
    hose_puller.move_z_actuator(z_home)
    puller_extension.close_gripper()
    hose_puller.move_y_actuator(pickup_y-500)
    hose_puller.move_z_actuator(safe_position)
    hose_puller.move_y_actuator(wait_y)
    insertion_servos.activate_cutter()
    hose_puller.move_y_actuator(home_y)
    hose_puller.move_z_actuator(z_home)
    hose_jig.gripper_close()
    puller_extension.open_gripper()
    hose_puller.move_z_actuator(safe_position)
    hose_jig.deliver_position()
    hose_jig.gripper_open()
    hose_puller.move_z_actuator(0)
    
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
    global insertion_jig, insertion_servos, lubrication_feeder

    # insertion_servos.slider_joint_home()

    #****************************** Insertion Jig ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4200 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 500 + offset_z
    lubricate_nozzle = -5380 + offset_x

    insertion_position_z = 490 + offset_z
    insert_nozzle = -6570 + offset_x  

    librication_position_joint_z = 580 + offset_z
    lubricate_joint = -11000 + offset_x

    insertion_position_joint_z = 600 + offset_z
    insert_joint = -8950 + offset_x

    #****************************** Routine ******************************

    #Home

    # insertion_servos.slider_joint_home()
    # insertion_servos.slider_nozzle_home()

    # insertion_jig.home_x_axis_go_home()
    # insertion_jig.home_z_axis_go_home()

    # insertion_servos.clamp_nozzle_open()
    # insertion_servos.clamp_joint_open()
    # insertion_servos.cutter_open()
    # insertion_servos.holder_hose_joint_open()
    # insertion_servos.holder_hose_nozzle_open()

    #Nozzle Position

    
    # insertion_jig.move_z_axis(home_position_z)
    # insertion_jig.move_x_axis(home_position_x)
    # insertion_servos.holder_hose_nozzle_close()

    # insertion_jig.move_x_axis(lubricate_nozzle)
    # insertion_jig.move_z_axis(lubrication_position_z)

    # lubrication_feeder.lubricate_nozzle(50)

    # time.sleep(.5)

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


    # # #Moving to Joint

    # insertion_servos.clamp_joint_open()
    # insertion_servos.holder_hose_joint_open()

    # return "ok"


    # insertion_servos.clamp_joint_close()

    # insertion_jig.move_z_axis(home_position_z)
    # insertion_jig.move_x_axis(home_position_x)
    

    # insertion_jig.move_x_axis(lubricate_joint)
    # insertion_jig.move_z_axis(librication_position_joint_z)

    # insertion_servos.holder_hose_joint_close()

    # lubrication_feeder.lubricate_joint(15)

    # time.sleep(.5)

    # insertion_jig.move_z_axis(insertion_position_joint_z)
    # insertion_jig.move_x_axis(insert_joint)
   

    # # # #Clamp Insertion

    # insertion_servos.holder_hose_joint_close()
    # insertion_servos.clamp_joint_close()
    # time.sleep(0.5)
    # insertion_servos.slider_joint_insertion()
    # time.sleep(1)
    insertion_servos.holder_hose_joint_open()
    insertion_servos.clamp_joint_open()
    time.sleep(0.5)
    insertion_servos.slider_joint_home()


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
    
    # elevator_in.home_gantry_z()
    # elevator_in.home_gantry_x()
    # elevator_in.home_gantry_y()

    #Left Side
    
    # elevator_in.move_gantry_array(pick_left)

    # elevator_in.close_gripper()

    # elevator_in.move_gantry_z(transportation_high)

    # elevator_in.move_gantry_array(deliver_left)

    # elevator_in.move_gantry_z(cassette_deliver_high)

    # elevator_in.open_gripper()

    # elevator_in.move_gantry_z(safety_high)

    # elevator_in.move_gantry_array(home)

    # # #Right Side

    # elevator_in.move_gantry_array(pick_right)

    # elevator_in.close_gripper()

    # elevator_in.move_gantry_z(transportation_high)

    # elevator_in.move_gantry_array(deliver_right)

    # elevator_in.move_gantry_z(cassette_deliver_high)

    # elevator_in.open_gripper()

    # elevator_in.move_gantry_z(safety_high)

    # elevator_in.move_gantry_array(home)

    # # #conveyors

    # pick_and_place.start_left_conveyor()
    # pick_and_place.start_right_conveyor()

    # time.sleep(5)

    # pick_and_place.stop_left_conveyor()
    # pick_and_place.stop_right_conveyor()

def movePickandPlace():
    global pick_and_place, insertion_servos, insertion_jig

    # pick_and_place.home_x_axis()
    # pick_and_place.home_z_axis()


    # Homing Insertion Jig
    insertion_servos.slider_joint_home()
    insertion_jig.move_z_axis(7000)
    insertion_jig.move_x_axis(-8500)
    pick_and_place.open_gripper()


    nozzle_high = -1312
    nozzle_x = [-615,-485,-335,-205,-55,75]
    trans_nozzle_high = -1000
    deliver_nozzle_x = -3900
    deliver_nozzle_z = -1325

    # *******************************************************

    # pick_and_place.start_left_conveyor()
    # time.sleep(.7)
    # pick_and_place.stop_left_conveyor()

    # #Home Pick and Place

    # for nozzle in nozzle_x:

    pick_and_place.move_actuator_z(0)
    pick_and_place.move_actuator_x(0)

    pick_and_place.move_actuator_x(nozzle_x[1])
    pick_and_place.move_actuator_z(nozzle_high)

    pick_and_place.close_gripper()

    pick_and_place.move_actuator_z(trans_nozzle_high)
    pick_and_place.move_actuator_x(deliver_nozzle_x)
    pick_and_place.move_actuator_z(deliver_nozzle_z)
    pick_and_place.open_gripper()

    pick_and_place.move_actuator_z(0,False)
    pick_and_place.move_actuator_x(0,False)   

    # *******************************************************


    # # #Pick Nozzle
    # pick_and_place.move_actuator_x(-1800)
    # pick_and_place.move_actuator_z(-1250)
    # pick_and_place.close_gripper()

    # # #Safe Position 1
    # pick_and_place.move_actuator_z(-1000)

    # # #Place Nozzle
    # pick_and_place.move_actuator_x(-4090)
    # pick_and_place.move_actuator_z(-1380)
    # pick_and_place.open_gripper()

    # #Safe Position 2
    # pick_and_place.move_actuator_z(-1000)

    # # #Pick Joint
    # pick_and_place.move_actuator_x(-850)
    # pick_and_place.move_actuator_z(-1250)
    # pick_and_place.close_gripper()

    # #Safe Position 3
    # pick_and_place.move_actuator_z(-1000)

    # # Place Joint
    # pick_and_place.move_actuator_x(-4390)
    # pick_and_place.move_actuator_z(-1320)
    # pick_and_place.open_gripper()

    # pick_and_place.move_actuator_z(0)
    # pick_and_place.move_actuator_x(0)
    # pick_and_place.open_gripper()

def oneCycle():

    global insertion_jig, insertion_servos, hose_puller, hose_jig, puller_extension,pick_and_place,lubrication_feeder


    # if lubrication_feeder.open_hose_holder() != "success" : return "error03"
    # return ""

    #****************************** Insertion Jig ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4000 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 500 + offset_z
    lubricate_nozzle = -5380 + offset_x

    insertion_position_z = 490 + offset_z
    insert_nozzle = -6570 + offset_x  

    librication_position_joint_z = 580 + offset_z
    lubricate_joint = -11000 + offset_x

    insertion_position_joint_z = 600 + offset_z
    insert_joint = -8950 + offset_x

    safe_position = 200
    home_y = 4200
    wait_y = 5930
    pickup_y = 8990
    z_home = 50

    #****************************** Routine ******************************



    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    if lubrication_feeder.feed_hose(duration=3.15) != "success" : return "error02"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error08"
    if lubrication_feeder.open_hose_holder() != "success" : return "error03"


    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    if puller_extension.open_gripper() != "success" : return "error20"

    if insertion_servos.slider_joint_home() != "success" : return "error08"
    if insertion_servos.slider_nozzle_home() != "success" : return "error08"

    # if insertion_jig.home_x_axis_go_home() != "success" : return "error08"
    # if insertion_jig.home_z_axis_go_home() != "success" : return "error08"

    #Nozzle Position

    if hose_jig.insertion_position(False) != "success" : return "error05"

    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error02"

    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error18"

    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error04"

    if lubrication_feeder.lubricate_nozzle(15) != "success" : return "error03.1"
    time.sleep(0.5)

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

    if insertion_jig.move_z_axis(4000) != "success" : return "error13"

    #Hose Puller Action

    if lubrication_feeder.move_pre_feeder(50) != "success" : return "error04"

    if hose_puller.move_z_actuator(safe_position) != "success" : return "error04"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error03"
    if hose_jig.insertion_position() != "success" : return "error05"
    if hose_puller.move_y_actuator(pickup_y+30) != "success" : return "error06"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error07"
    if puller_extension.close_gripper() != "success" : return "error08"
    if hose_puller.move_y_actuator(pickup_y-500) != "success" : return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error10"
    if hose_puller.move_y_actuator(wait_y+1900) != "success" : return "error11"
    if hose_puller.move_z_actuator(safe_position+35) != "success" : return "error10"
    if hose_puller.move_y_actuator(wait_y) != "success" : return "error11"
    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success" : return "error12"
    if hose_puller.move_y_actuator_with_speed(home_y+660,100) != "success" : return "error13" #Aqui ahre el cambio
    
    if lubrication_feeder.move_pre_feeder(0) != "success" : return "error04"

    #Moving to Joint

    if insertion_servos.holder_hose_joint_close() != "success" : return "error18"

    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error15"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error14"

    if lubrication_feeder.lubricate_joint(16) != "success" : return "error14.1"
    time.sleep(0.5)

    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error17"
    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success" : return "error16"
   

    #Joint Insertion

   
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

    if insertion_jig.move_z_axis(3000,False) != "success" : return "error24"
    if insertion_jig.move_x_axis(0,False) != "success" : return "error25"


    if hose_puller.move_z_actuator(z_home) != "success" : return "error14"
    if hose_jig.gripper_close() != "success" : return "error15"
    if puller_extension.open_gripper() != "success" : return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error17"
    if hose_jig.deliver_position() != "success" : return "error18"
    if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success" : return "error20"

    return "success"

def insertionAccuracy():
    global insertion_jig,insertion_servos,pick_and_place

    # insertion_servos.holder_hose_nozzle_close()
    # if insertion_servos.slider_joint_home() != "success" : return "error22"
    # if insertion_jig.move_x_axis(-1370) != "success" : return "error23"
    # if insertion_jig.home_x_axis_go_home() != "success" : return "error24"
    # for i in range(10):

    # pick_and_place.start_left_conveyor()
    # pick_and_place.start_right_conveyor()
    # for i in range(10):
    #     if insertion_jig.move_z_axis(0) != "success" : return "error24"
    #     if insertion_jig.move_z_axis(9000) != "success" : return "error24"
        
    # if C != "success" : return "error24"

    # if insertion_jig.move_z_axis(5000) != "success" : return "error24"
        
    # pick_and_place.stop_left_conveyor()
    # pick_and_place.stop_right_conveyor()

def testHome():
    # """Test function for new go home methods"""
    global elevator_in, pick_and_place, hose_puller, hose_jig, insertion_jig
    
    print("Testing new go home functions...")
    
    # Test ElevatorIn homing functions
    print("\n--- Testing ElevatorIn homing functions ---")
    
    print("Testing home_gantry_x...")
    result = elevator_in.home_gantry_x()
    print(f"Result: {result}")
    
    print("Testing home_gantry_y...")
    result = elevator_in.home_gantry_y()
    print(f"Result: {result}")
    
    print("Testing home_gantry_z...")
    result = elevator_in.home_gantry_z()
    print(f"Result: {result}")
    
    # print("Testing home_elevator_z...")
    # result = elevator_in.home_elevator_z()
    # print(f"Result: {result}")

    # input("Continue") 
    
    # Test PickAndPlace homing functions
    print("\n--- Testing PickAndPlace homing functions ---")
    
    print("Testing home_x_axis...")
    result = pick_and_place.home_x_axis()
    print(f"Result: {result}")
    
    print("Testing home_z_axis...")
    result = pick_and_place.home_z_axis()
    print(f"Result: {result}")

    # input("Continue")

    # Test HoseJig homing function
    print("\n--- Testing HoseJig homing function ---")
    
    print("Testing home_actuator...")
    result = hose_jig.go_home()
    print(f"Result: {result}")
    
    
    # input("Continue") 

    # Test HosePuller homing functions
    print("\n--- Testing HosePuller homing functions ---")
    
    print("Testing home_y_axis...")
    result = hose_puller.home_y_axis()
    print(f"Result: {result}")
    
    print("Testing home_z_axis...")
    result = hose_puller.home_z_axis()
    print(f"Result: {result}")
    
    # input("Continue")

    # Test InsertionJig homing functions
    print("\n--- Testing InsertionJig homing functions ---")
    
    print("Testing home_x_axis...")
    result = insertion_jig.home_x_axis_go_home()
    print(f"Result: {result}")
    
    print("Testing home_z_axis...")
    result = insertion_jig.home_z_axis_go_home()
    print(f"Result: {result}")
    
    print("\nAll homing tests completed!")

def lubrication_test():
    global lubrication_feeder,insertion_servos

    # for i in range(1):
    # lubrication_feeder.lubricate_nozzle(duration=5)
    # lubrication_feeder.lubricate_joint(duration=6)
    #     time.sleep(.5)

    # lubrication_feeder.open_hose_holder()
    # return""
    # lubrication_feeder.close_hose_holder()
    # lubrication_feeder.feed_hose(duration=3.15)
    # lubrication_feeder.close_hose_holder()
    # time.sleep(1)
    # insertion_servos.activate_cutter()


if __name__ == "__main__":
   
    my_main()

    # insertionAccuracy()

    # moveElevatorIn()

    # movePickandPlace()

    # insertionRoutine()

    # moveHosepuller()
    
    # Test the new go home functions
    # testHome()
    
    # for i in range(1):
    oneCycle()

    # lubrication_test()

    canbus.close_canbus()


