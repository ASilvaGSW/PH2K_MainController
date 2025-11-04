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
from classes.lubrication_stamper import LubricationStamper
from classes.stamper import Stamper
from classes.taping import Taping
from classes.taping_fuyus import TapingFuyus
from classes.transporter_fuyus import TransporterFuyus
from classes.transporter_grippers import TransporterGrippers

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
lubrication_stamper = None
stamper = None
taping = None
taping_fuyus = None
transporter_fuyus = None
transporter_grippers = None

# Connectivity flags for different stations
CHECK_FIRST_STATION = False
CHECK_SECOND_STATION = True
CHECK_ALL_DEVICES = False  # When True, checks all devices regardless of station flags

#Initialize and setup CANbus and devices
def my_main():
    global canbus, hose_jig, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos
    global lubrication_feeder, lubrication_stamper, stamper, taping, taping_fuyus, transporter_fuyus, transporter_grippers
    
    # Example CAN IDs (these should be configured according to your hardware setup)
    CANBUS_ID_JIG = 0x0CA
    CANBUS_ID_PULLER = 0x192
    CANBUS_ID_EXTENSION = 0x193
    CANBUS_ID_INSERTION = 0x0C9
    CANBUS_ID_ELEVATOR_IN = 0x189   
    CANBUS_ID_PICK_AND_PLACE = 0x191
    CANBUS_ID_INSERTION_SERVOS = 0x002
    CANBUS_ID_LUBRICATION_FEEDER = 0x019
    CANBUS_ID_LUBRICATION_STAMPER = 0x004
    CANBUS_ID_STAMPER = 0x005
    CANBUS_ID_TAPING = 0x00A
    CANBUS_ID_TAPING_FUYUS = 0x007
    CANBUS_ID_TRANSPORTER_FUYUS = 0x021
    CANBUS_ID_TRANSPORTER_GRIPPERS = 0x020

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
    lubrication_stamper = LubricationStamper(canbus, CANBUS_ID_LUBRICATION_STAMPER)
    stamper = Stamper(canbus, CANBUS_ID_STAMPER)
    taping = Taping(canbus, CANBUS_ID_TAPING)
    taping_fuyus = TapingFuyus(canbus, CANBUS_ID_TAPING_FUYUS)
    transporter_fuyus = TransporterFuyus(canbus, CANBUS_ID_TRANSPORTER_FUYUS)
    transporter_grippers = TransporterGrippers(canbus, CANBUS_ID_TRANSPORTER_GRIPPERS)

    checkConnectivity()


#Check connectivity
def checkConnectivity():
    
    global canbus, hose_jig, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos
    global lubrication_feeder, lubrication_stamper, stamper, transporter_fuyus, transporter_grippers
    global CHECK_FIRST_STATION, CHECK_SECOND_STATION, CHECK_ALL_DEVICES

    # First Station - Check only if flag is enabled or CHECK_ALL_DEVICES is True
    if CHECK_FIRST_STATION or CHECK_ALL_DEVICES:
        print("=== CHECKING FIRST STATION ===")
        print("Hose Jig Connected") if hose_jig.send_heartbeat() == "success" else print("Hose Jig Not Connected")
        print("Hose Puller Connected") if hose_puller.send_heartbeat() == "success" else print("Hose Puller Not Connected")
        print("Puller Extension Connected") if puller_extension.send_heartbeat() == "success" else print("Puller Extension Not Connected")
        print("Pick and Place Connected") if pick_and_place.send_heartbeat() == "success" else print("Pick and Place Not Connected")
        print("Insertion Jig Connected") if insertion_jig.send_heartbeat() == "success" else print("Insertion Jig Not Connected")
        print("Elevator In Connected") if elevator_in.send_heartbeat() == "success" else print("Elevator In Not Connected")
        print("Insertion Servos Connected") if insertion_servos.send_heartbeat() == "success" else print("Insertion Servos Not Connected")
        print("Lubrication Feeder Connected") if lubrication_feeder.send_heartbeat() == "success" else print("Lubrication Feeder Not Connected")
    
    # Second Station - Check only if flag is enabled or CHECK_ALL_DEVICES is True
    if CHECK_SECOND_STATION or CHECK_ALL_DEVICES:
        print("=== CHECKING SECOND STATION ===")
        # print("Lubrication Stamper Connected") if lubrication_stamper.send_heartbeat() == "success" else print("Lubrication Stamper Not Connected")
        # print("Stamper Connected") if stamper.send_heartbeat() == "success" else print("Stamper Not Connected")
        print("Taping Connected") if taping.send_heartbeat() == "success" else print("Taping Not Connected")
        print("Taping Fuyus Connected") if taping_fuyus.send_heartbeat() == "success" else print("Taping Fuyus Not Connected")
        # print("Transporter Fuyus Connected") if transporter_fuyus.send_heartbeat() == "success" else print("Transporter Fuyus Not Connected")
        # print("Transporter Grippers Connected") if transporter_grippers.send_heartbeat() == "success" else print("Transporter Grippers Not Connected")

    # Show current connectivity check configuration
    if not CHECK_ALL_DEVICES:
        print(f"\n=== CONNECTIVITY CHECK CONFIG ===")
        print(f"First Station: {'ENABLED' if CHECK_FIRST_STATION else 'DISABLED'}")
        print(f"Second Station: {'ENABLED' if CHECK_SECOND_STATION else 'DISABLED'}")
    else:
        print(f"\n=== CONNECTIVITY CHECK CONFIG ===")
        print("Checking ALL DEVICES (station flags ignored)")


#Move Hose Puller
def moveHosepuller():
    global hose_puller, hose_jig, puller_extension,insertion_servos, insertion_jig

    # Variables 

    safe_position = 200
    home_y = 4200
    wait_y = 5930
    pickup_y = 8990
    z_home = 50

    if insertion_jig.move_z_axis(7000) != "success": return "error01"
    if insertion_jig.move_x_axis(-8500) != "success": return "error02"

    #****************************** Hose puller ******************************
    if hose_puller.home_y_axis() != "success": return "error03"
    if hose_puller.home_z_axis() != "success": return "error04"

    if hose_jig.go_home() != "success": return "error05"

    if hose_puller.move_y_actuator(home_y) != "success": return "error06"
    if hose_puller.move_z_actuator(safe_position) != "success": return "error07"
    if hose_jig.insertion_position() != "success": return "error08"
    if hose_puller.move_y_actuator(pickup_y) != "success": return "error09"
    if hose_puller.move_z_actuator(z_home) != "success": return "error10"
    if puller_extension.close_gripper() != "success": return "error11"
    if hose_puller.move_y_actuator(pickup_y-500) != "success": return "error12"
    if hose_puller.move_z_actuator(safe_position) != "success": return "error13"
    if hose_puller.move_y_actuator(wait_y) != "success": return "error14"
    if insertion_servos.activate_cutter() != "success": return "error15"
    if hose_puller.move_y_actuator(home_y) != "success": return "error16"
    if hose_puller.move_z_actuator(z_home) != "success": return "error17"
    if hose_jig.gripper_close() != "success": return "error18"
    if puller_extension.open_gripper() != "success": return "error19"
    if hose_puller.move_z_actuator(safe_position) != "success": return "error20"
    if hose_jig.deliver_position() != "success": return "error21"
    if hose_jig.gripper_open() != "success": return "error22"
    if hose_puller.move_z_actuator(0) != "success": return "error23"

    return "success"


#Insertion Routine
def insertionRoutine():

    global insertion_jig, insertion_servos, lubrication_feeder

    #****************************** Insertion Jig ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4200 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 520 + offset_z
    lubricate_nozzle = -5080 + offset_x

    insertion_position_z = 480 + offset_z
    insert_nozzle = -6570 + offset_x  

    librication_position_joint_z = 530 + offset_z
    lubricate_joint = -11170 + offset_x

    insertion_position_joint_z = 600 + offset_z
    insert_joint = -8950 + offset_x

    #****************************** Routine ******************************
    
    # Homing Devices
    
    if insertion_servos.slider_joint_home() != "success": return "error04"
    if insertion_servos.slider_nozzle_home() != "success": return "error05"
    
    if insertion_jig.home_x_axis_go_home() != "success": return "error06"
    if insertion_jig.home_z_axis_go_home() != "success": return "error07"
    
    if insertion_servos.clamp_nozzle_open() != "success": return "error09"
    if insertion_servos.clamp_joint_open() != "success": return "error10"
    if insertion_servos.cutter_open() != "success": return "error11"
    if insertion_servos.holder_hose_joint_open() != "success": return "error12"
    if insertion_servos.holder_hose_nozzle_open() != "success": return "error13"
    
    # Feed Hose
    
    if lubrication_feeder.close_hose_holder() != "success": return "error01"
    if lubrication_feeder.feed_hose(duration=3.15) != "success": return "error02"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_close() != "success": return "error08"
    if lubrication_feeder.open_hose_holder() != "success": return "error03"
    
    
    # Align Nozzle Position
    
    
    if insertion_jig.move_z_axis(home_position_z) != "success": return "error14"
    if insertion_jig.move_x_axis(home_position_x) != "success": return "error15"
    
    # Lubricate Hose Nozzle Area
    
    if insertion_servos.holder_hose_nozzle_close() != "success": return "error16"
    
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success": return "error17"
    if insertion_jig.move_z_axis(lubrication_position_z) != "success": return "error18"
    
    if lubrication_feeder.lubricate_nozzle(5) != "success": return "error19"
    
    # Insert Nozzle
    
    if insertion_jig.move_z_axis(insertion_position_z) != "success": return "error20"
    if insertion_jig.move_x_axis(insert_nozzle) != "success": return "error21"
    
    if insertion_servos.holder_hose_nozzle_close() != "success": return "error22"
    if insertion_servos.clamp_nozzle_close() != "success": return "error23"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success": return "error24"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success": return "error25"
    if insertion_servos.clamp_nozzle_open() != "success": return "error26"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_home() != "success": return "error27"
    
    
    # Safe Position
    
    
    if insertion_servos.clamp_joint_close() != "success": return "error28"
    if insertion_jig.move_z_axis(home_position_z) != "success": return "error29"
    
    # Lubricate Hose Joint Area
    
    if insertion_jig.move_x_axis(lubricate_joint) != "success": return "error30"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success": return "error31"
    if lubrication_feeder.lubricate_joint(5) != "success": return "error32"
    
    #Insert Joint
    
    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success": return "error33"
    if insertion_jig.move_x_axis(insert_joint) != "success": return "error34"
    if insertion_servos.holder_hose_joint_close() != "success": return "error35"
    if insertion_servos.clamp_joint_close() != "success": return "error36"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success": return "error37"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success": return "error38"
    if insertion_servos.clamp_joint_open() != "success": return "error39"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success": return "error40"
    
    # Back to Home
    
    if insertion_jig.move_z_axis(0) != "success": return "error41"
    if insertion_jig.move_x_axis(0) != "success": return "error42"

    if insertion_servos.activate_cutter() != "success": return "error43"

    return "success"


#Open All Servos
def insertionServosOpen():
    global insertion_servos

    insertion_servos.slider_joint_home()
    insertion_servos.slider_nozzle_home()
    insertion_servos.clamp_nozzle_open()
    insertion_servos.clamp_joint_open()
    insertion_servos.cutter_open()
    insertion_servos.holder_hose_joint_open()
    insertion_servos.holder_hose_nozzle_open()


#Close All Servos
def insertionServosClose():
    global insertion_servos

    insertion_servos.clamp_nozzle_close()
    insertion_servos.clamp_joint_close()
    insertion_servos.cutter_close()
    insertion_servos.holder_hose_joint_close()
    insertion_servos.holder_hose_nozzle_close()


#Move Elevator In
def moveElevatorIn():

    global elevator_in,pick_and_place

    home = [0,0,0]
    safety_high = -300
    cassette_deliver_high = -610
    transportation_high = -500

    #left
    pick_left = [-225,-315,-820]
    deliver_left = [-225,-1800,-500]
    
    #Right
    pick_right = [-1358,-317,-850]
    deliver_right = [-1353,-1800,-500]


    #Homing
    # if elevator_in.home_gantry_z() != "success": return "error01"
    # if elevator_in.home_gantry_x() != "success": return "error02"
    # if elevator_in.home_gantry_y() != "success": return "error03"
    # if elevator_in.home_elevator_z() != "success": return "error04"

    #Alignment For 4 Cassettes, Will change by IR Sensor on Saturday
    # if elevator_in.move_elevator_z(-12500) != "success": return "error05"
    if elevator_in.move_elevator_until_sensor(0, 200) != "success": return "error05"


    #Left Side
    if elevator_in.move_gantry_array(pick_left) != "success": return "error06"
    if elevator_in.close_gripper() != "success": return "error07"
    if elevator_in.move_gantry_z(transportation_high) != "success": return "error08"
    if elevator_in.move_gantry_array(deliver_left) != "success": return "error09"
    if elevator_in.move_gantry_z(cassette_deliver_high) != "success": return "error10"
    if elevator_in.open_gripper() != "success": return "error11"
    if elevator_in.move_gantry_z(safety_high) != "success": return "error12"
    if elevator_in.move_gantry_array(home) != "success": return "error13"

    #Right Side
    if elevator_in.move_gantry_array(pick_right) != "success": return "error14"
    if elevator_in.close_gripper() != "success": return "error15"
    if elevator_in.move_gantry_z(transportation_high) != "success": return "error16"
    if elevator_in.move_gantry_array(deliver_right) != "success": return "error17"
    if elevator_in.move_gantry_z(cassette_deliver_high) != "success": return "error18"
    if elevator_in.open_gripper() != "success": return "error19"
    if elevator_in.move_gantry_z(safety_high) != "success": return "error20"
    if elevator_in.move_gantry_array(home) != "success": return "error21"

    #conveyors
    if pick_and_place.start_left_conveyor() != "success": return "error22"
    if pick_and_place.start_right_conveyor() != "success": return "error23"
    time.sleep(3)
    if pick_and_place.stop_left_conveyor() != "success": return "error24"
    if pick_and_place.stop_right_conveyor() != "success": return "error25"

    alignCassette()

    return "success"


#Map Nozzle Cassette
def mapNozzleCassette():

    global pick_and_place

    for i in range(5):
        
        for j in range(2):
        
            pick_and_place.move_actuator_z(0)
            pick_and_place.move_actuator_x(0)
        
            pick_and_place.move_actuator_x(nozzle_x[j])
            pick_and_place.move_actuator_z(nozzle_high)
        
        pick_and_place.move_actuator_z(0)
        pick_and_place.move_actuator_x(0)
        
        pick_and_place.start_left_conveyor()
        time.sleep(.73)
        pick_and_place.stop_left_conveyor()


#Move Pick and Place
def movePickandPlace(n=1):
    global pick_and_place, insertion_servos, insertion_jig

    receiving_x = -8500
    receiving_y = 7000
    translation_gap = 0.73

    # Homing Pick and Place
    # if pick_and_place.home_z_axis() != "success": return "error01"
    # if pick_and_place.home_x_axis() != "success": return "error02"

    # Homing Insertion Jig
    if insertion_servos.slider_joint_receive() != "success": return "error03"
    if insertion_servos.slider_nozzle_receive() != "success": return "error3.1"
    if insertion_jig.move_z_axis(receiving_y) != "success": return "error04"
    if insertion_jig.move_x_axis(receiving_x) != "success": return "error05"
    if pick_and_place.open_gripper() != "success": return "error06"

    gap = 15
    zgap = 0

    #Nozzle Data
    nozzle_high = -1255
    nozzle_x = [-640,-502,-347,-220,-70]
    trans_nozzle_high = -1000
    deliver_nozzle_x = -3900
    deliver_nozzle_z = -1248

    # Joint Data
    joint_high = -1243
    joint_x = [-1800,-1715,-1635,-1555,-1475]
    trans_joint_high = -1000
    deliver_joint_x = -4190
    deliver_joint_z = -1262


    # Validate Home

    if pick_and_place.move_actuator_z(0) != "success": return "error07"
    if pick_and_place.move_actuator_x(0) != "success": return "error08"

    #
    # Pick Nozzle
    if pick_and_place.move_actuator_x(nozzle_x[n]- gap) != "success": return "error09"
    if pick_and_place.move_actuator_z(nozzle_high+zgap) != "success": return "error10"
    if pick_and_place.close_gripper() != "success": return "error11"


    # Deliver Nozzle
    if pick_and_place.move_actuator_z(trans_nozzle_high) != "success": return "error12"
    if pick_and_place.move_actuator_x(deliver_nozzle_x - gap) != "success": return "error13"
    if pick_and_place.move_actuator_z(deliver_nozzle_z+zgap) != "success": return "error14"
    if pick_and_place.open_gripper() != "success": return "error15"
    time.sleep(.5)


    # Back to Home
    if pick_and_place.move_actuator_z(0,False) != "success": return "error16"
    if pick_and_place.move_actuator_x(0,False) != "success": return "error17"


    # *******************************************************

    # Joint Pick Up

    # Home (ensure is there before moving)
    if pick_and_place.move_actuator_z(0) != "success": return "error18"
    if pick_and_place.move_actuator_x(0) != "success": return "error19"

    # Pick Joint
    if pick_and_place.move_actuator_x(joint_x[n]-gap) != "success": return "error21"
    if pick_and_place.move_actuator_z(joint_high+zgap) != "success": return "error22"
    if pick_and_place.close_gripper() != "success": return "error23"

    # Deliver Joint
    if pick_and_place.move_actuator_z(trans_joint_high) != "success": return "error24"
    if pick_and_place.move_actuator_x(deliver_joint_x-gap) != "success": return "error25"
    if pick_and_place.move_actuator_z(deliver_joint_z+zgap) != "success": return "error26"
    if pick_and_place.open_gripper() != "success": return "error27"

    # Go Back to Home
    if pick_and_place.move_actuator_z(0,False) != "success": return "error28"
    if pick_and_place.move_actuator_x(0,False) != "success": return "error29"

    return "success"


#Main One Cycle
def oneCycle():

    global insertion_jig, insertion_servos, hose_puller, hose_jig
    global puller_extension,pick_and_place,lubrication_feeder

    #****************************** Insertion Jig Data ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4200 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 520 + offset_z
    lubricate_nozzle = -5080 + offset_x

    insertion_position_z = 498 + offset_z
    insert_nozzle = -6570 + offset_x

    librication_position_joint_z = 530 + offset_z
    lubricate_joint = -11170 + offset_x

    insertion_position_joint_z = 600 + offset_z
    insert_joint = -8950 + offset_x

    #****************************** Hose Puller Data ******************************

    safe_position = 200
    safe_position_over_hose_jig = 242
    home_y = 4200
    wait_y = 5930
    cutting_position = 7830
    pickup_y = 9005
    before_rise_position = 8510
    z_home = 50
    z_picking_position = 55
    alignmnet_for_joint = 4865

    #****************************** Custom Variables ******************************

    insertion_jig_safe_zone = 4000
    preefeder_speed = 50
    feed_hose_time = 3.15
    lubricate_nozzle_time = 5
    lubricate_joint_time = 5
    hose_puller_y_speed = 200
    hose_puller_y_speed_for_alignment = 100

    #****************************** Routine ******************************

    # Feed Hose

    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    if lubrication_feeder.feed_hose(duration=feed_hose_time) != "success" : return "error02"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error08"
    if lubrication_feeder.open_hose_holder() != "success" : return "error03"

    # Clamping and Reset Gripper
    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    if puller_extension.open_gripper() != "success" : return "error20"

    # Homing Slider Before Movement
    if insertion_servos.slider_joint_home() != "success" : return "error08"
    if insertion_servos.slider_nozzle_home() != "success" : return "error08"

    # Insertio Jig Home POsition

    if hose_jig.insertion_position(False) != "success" : return "error05"
    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error02"


    #Lubricate Hose

    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error18"
    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error04"
    if lubrication_feeder.lubricate_nozzle(lubricate_nozzle_time) != "success" : return "error03.1"

    # Nozzle Insertion
    if insertion_jig.move_z_axis(insertion_position_z) != "success" : return "error05"
    if insertion_jig.move_x_axis(insert_nozzle) != "success" : return "error06"
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success" : return "error09"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success" : return "error10"
    if insertion_servos.clamp_nozzle_open() != "success" : return "error11"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_home() != "success" : return "error12"

    # Go to Down Position for Hose Puller
    if insertion_jig.move_z_axis(insertion_jig_safe_zone) != "success" : return "error13"
    #
    # # Starting Prefeeder
    if lubrication_feeder.move_pre_feeder(preefeder_speed) != "success" : return "error04"

    # Preparing Hose Puller and Hose Jig
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error04"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error03"
    if hose_jig.insertion_position() != "success" : return "error05"

    # Pulling Action
    if hose_puller.move_y_actuator(pickup_y) != "success" : return "error06"
    if hose_puller.move_z_actuator(z_picking_position) != "success" : return "error07"
    if puller_extension.close_gripper() != "success" : return "error08"
    if hose_puller.move_y_actuator_with_speed(before_rise_position,hose_puller_y_speed) != "success" : return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error10"
    if hose_puller.move_y_actuator_with_speed(cutting_position,hose_puller_y_speed) != "success" : return "error11"
    if hose_puller.move_z_actuator(safe_position_over_hose_jig) != "success" : return "error10"
    if hose_puller.move_y_actuator_with_speed(wait_y,hose_puller_y_speed) != "success" : return "error11"

    # Cutting Hose
    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success" : return "error12"

    # Alignment for Joint Insertion
    if hose_puller.move_y_actuator_with_speed(alignmnet_for_joint,hose_puller_y_speed_for_alignment) != "success" : return "error13" #Aqui ahre el cambio
    
    # Stoping Prefeeder
    if lubrication_feeder.move_pre_feeder(0) != "success" : return "error04"

    # Lubricate Hose on Joint Area
    if insertion_servos.holder_hose_joint_close() != "success" : return "error18"
    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error15"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error14"
    if lubrication_feeder.lubricate_joint(lubricate_joint_time) != "success" : return "error14.1"
    time.sleep(0.5)

    # Joint Insertion
    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error17"
    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success" : return "error16"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success" : return "error20"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success" : return "error21"
    if insertion_servos.clamp_joint_open() != "success" : return "error22"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success" : return "error23"


    # Finish Pulling Action

    if hose_puller.move_y_actuator(home_y) != "success" : return "error13"

    # Homing for Finish
    if insertion_jig.move_z_axis(insertion_jig_safe_zone,False) != "success" : return "error24"
    if insertion_jig.move_x_axis(0,False) != "success" : return "error25"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error14"

    # Delivering Hose

    if hose_jig.gripper_close() != "success" : return "error15"
    if puller_extension.open_gripper() != "success" : return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error17"
    if hose_jig.deliver_position() != "success" : return "error18"
    # if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success" : return "error20"

    return "success"


#Test transporter and grippers
def moveTransporter():
    global transporter_fuyus, transporter_grippers,hose_jig

    # print(hose_jig.read_deliver())
    
    # Test TransporterFuyus functionality
    # print("\n--- Testing TransporterFuyus ---")
    
    # Home X axis
    # print("Homing X axis...")
    # result = transporter_fuyus.home_x_axis()
    # print(f"Result: {result}")
    
    # Move X axis to different positions
    # print("Moving X axis to Taping Hose Jig ")
    # result = transporter_fuyus.moveToTapingHoseJig()
    # print(f"Result: {result}")

    # print("Moving X axis to Stamper Hose Jig ")
    # result = transporter_fuyus.moveToStamperHoseJig()
    # print(f"Result: {result}")

    # print("Moving X axis to Deliver ")
    # result = transporter_fuyus.moveToDeliverPosition()
    # print(f"Result: {result}")

    # print("Moving X axis to Receiving ")
    # result = transporter_fuyus.moveToReceivingPosition()
    # print(f"Result: {result}")

    
    # Move all steppers
    # Move all steppers
    # print("Moving all steppers to position 1000...")
    # result = transporter_fuyus.move_all_steppers(150000,1)
    # print(f"Result: {result}")

    # result = transporter_fuyus.pickHoseFromFirstStation()
    # print(f"Result: {result}")

    # result = transporter_fuyus.pickHome()
    # # Test TransporterGrippers functionality
    # print("\n--- Testing TransporterGrippers ---")
    
    # Open and close all grippers

    # print("Closing all grippers...")
    # result = transporter_grippers.close_all_grippers()
    # print(f"Result: {result}")

    # time.sleep(2)

    # print("Opening all grippers...")
    # result = transporter_grippers.open_all_grippers()
    # print(f"Result: {result}")


#Pick Hose
def pickUpHose():
    global transporter_fuyus, transporter_grippers, hose_jig

    if hose_jig.gripper_close() != "success" : return "001"

    if transporter_fuyus.pickHome() != "success" : return "01"
    # if transporter_fuyus.moveToStamperHoseJig() != "success" : return "02"
    if transporter_fuyus.moveToReceivingPosition() != "success" : return "03"

    if transporter_fuyus.pickHoseFromFirstStation() != "success" : return "04"
    if transporter_grippers.close_all_grippers() != "success" : return "05"

    time.sleep(1)

    if hose_jig.gripper_open() != "success" : return "06"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToDeliverPosition() != "success": return "03"

    if transporter_grippers.open_all_grippers() != "success": return "05"

    return "success"


#Test all home functions
def testHome():
    # """Test function for new go home methods"""
    global elevator_in, pick_and_place, hose_puller, hose_jig, insertion_jig, insertion_servos,transporter_fuyus

    insertion_servos.slider_joint_home()
    
    print("Testing new go home functions...")
    
    # Test ElevatorIn homing functions
    print("\n--- Testing ElevatorIn homing functions ---")

    print("Testing home_gantry_z...")
    result = elevator_in.home_gantry_z()
    print(f"Result: {result}")

    print("Testing home_gantry_x...")
    result = elevator_in.home_gantry_x()
    print(f"Result: {result}")
    
    print("Testing home_gantry_y...")
    result = elevator_in.home_gantry_y()
    print(f"Result: {result}")

    
    print("Testing home_elevator_z...")
    result = elevator_in.home_elevator_z()
    print(f"Result: {result}")

    # input("Continue")
    
    # Test PickAndPlace homing functions
    print("\n--- Testing PickAndPlace homing functions ---")

    result = pick_and_place.home_z_axis()
    print(f"Result: {result}")

    print("Testing home_x_axis...")
    result = pick_and_place.home_x_axis()
    print(f"Result: {result}")
    
    print("Testing home_z_axis...")

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

    # Test Transporter homing functions
    print("\n--- Testing Transporter homing functions ---")

    print("Testing home_x_axis...")
    result = transporter_fuyus.home_x_axis()
    print(f"Result: {result}")

    
    print("\nAll homing tests completed!")

#Test lubrication
def lubrication_test():
    global lubrication_feeder,insertion_servos,hose_puller

    # if lubrication_feeder.lubricate_nozzle(duration=5) != "success": return "error01"
    # if lubrication_feeder.lubricate_joint(duration=5) != "success": return "error02"

    hose_puller.move_y_actuator_with_speed(5,100)

    # if lubrication_feeder.set_hose_holder_angle(20) != "success": return "error01"
    # if lubrication_feeder.feed_hose(speed=1500,direction=0,duration=16) != "success": return "error01"
    # if lubrication_feeder.set_hose_holder_angle(0) != "success": return "error01"

    # if insertion_servos.activate_cutter() != "success": return "error02"


    return "success"


#Align Cassette:
def alignCassette():
    global pick_and_place



    if pick_and_place.move_left_conveyor_until_sensor(0,1000) != "success" : return "error01"
    if pick_and_place.move_right_conveyor_until_sensor(0,1000) != "success" : return "error02"
    # #
    # # pick_and_place.move_left_conveyor(0, 50, 236)
    # # pick_and_place.move_left_conveyor(0, 0, 236)
    #
    pick_and_place.move_right_conveyor(0, 50, 236)
    time.sleep(1.3)
    pick_and_place.move_right_conveyor(0, 0, 236)


def testIR():
    global elevator_in

    print(elevator_in.check_ir_sensor_status())

    print("Testing home_elevator_z...")
    result = elevator_in.home_elevator_z()
    print(f"Result: {result}")


# Helper functions to control connectivity flags
def set_connectivity_mode(first_station=None, second_station=None, all_devices=None):
    """
    Set connectivity check modes
    Args:
        first_station (bool): Enable/disable first station checking
        second_station (bool): Enable/disable second station checking  
        all_devices (bool): Enable/disable checking all devices (overrides station flags)
    """
    global CHECK_FIRST_STATION, CHECK_SECOND_STATION, CHECK_ALL_DEVICES
    
    if first_station is not None:
        CHECK_FIRST_STATION = first_station
    if second_station is not None:
        CHECK_SECOND_STATION = second_station
    if all_devices is not None:
        CHECK_ALL_DEVICES = all_devices
    
    print(f"Connectivity mode updated:")
    print(f"  First Station: {'ENABLED' if CHECK_FIRST_STATION else 'DISABLED'}")
    print(f"  Second Station: {'ENABLED' if CHECK_SECOND_STATION else 'DISABLED'}")
    print(f"  All Devices: {'ENABLED' if CHECK_ALL_DEVICES else 'DISABLED'}")


def enable_first_station_only():
    """Enable connectivity check for first station only"""
    set_connectivity_mode(first_station=True, second_station=False, all_devices=False)


def enable_second_station_only():
    """Enable connectivity check for second station only"""
    set_connectivity_mode(first_station=False, second_station=True, all_devices=False)


def enable_all_stations():
    """Enable connectivity check for all stations"""
    set_connectivity_mode(first_station=True, second_station=True, all_devices=False)


def enable_all_devices():
    """Enable connectivity check for all devices (ignores station flags)"""
    set_connectivity_mode(all_devices=True)


def disable_connectivity_checks():
    """Disable all connectivity checks"""
    set_connectivity_mode(first_station=False, second_station=False, all_devices=False)


def get_connectivity_status():
    """Get current connectivity check configuration"""
    global CHECK_FIRST_STATION, CHECK_SECOND_STATION, CHECK_ALL_DEVICES
    
    return {
        'first_station': CHECK_FIRST_STATION,
        'second_station': CHECK_SECOND_STATION,
        'all_devices': CHECK_ALL_DEVICES
    }



def taping_fuyus_test():
    """
    Función de prueba para la clase TapingFuyus
    Prueba las funciones principales de control de actuadores Y y Z
    """
    global taping_fuyus
    
    print("=== Iniciando prueba de TapingFuyus ===")
    
    # Enviar heartbeat
    # print("Enviando heartbeat...")
    # taping_fuyus.send_heartbeat()
    # time.sleep(0.5)    
    # Enviar Z a Home
    # print("Enviando Z a Home...")
    # taping_fuyus.home_z_actuator()
    # time.sleep(0.5)
    while True:
        taping_fuyus.move_y_actuator_with_speed(1000,300)
        taping_fuyus.move_z_actuator_with_speed(800,300)    
        taping_fuyus.move_z_actuator_with_speed(0,300)    
        taping_fuyus.move_y_actuator_with_speed(0,300)
        taping_fuyus.move_z_actuator_with_speed(800,300)    
        time.sleep(2)
        taping_fuyus.move_z_actuator_with_speed(0,300) 


    # print("Enviando Z a X...")
    # for i in range(1,10):
    #     taping_fuyus.move_z_actuator(0)
    #     time.sleep(0.5)
    #     taping_fuyus.move_z_actuator(810)
    #     time.sleep(0.5)

    # print("Enviando Z a X...")
    # for i in range(1,10):
    #     taping_fuyus.move_z_actuator_with_speed(0,300)
    #     time.sleep(0.5)
    #     taping_fuyus.move_z_actuator_with_speed(800,300)
    #     time.sleep(2.5)        
    
    # # Hacer home de todos los actuadores
    # print("Haciendo home de todos los actuadores...")
    # taping_fuyus.home_all_actuators()
    # time.sleep(2)
    
    # # Leer contadores iniciales
    # print("Leyendo contadores iniciales...")
    # y_counter = taping_fuyus.read_y_counter()
    # z_counter = taping_fuyus.read_z_counter()
    # print(f"Contador Y: {y_counter}, Contador Z: {z_counter}")
    
    # # Mover actuador Y a posición absoluta
    # print("Moviendo actuador Y a posición 1000...")
    # taping_fuyus.move_y_absolute(1000)
    # time.sleep(1)
    
    # # Mover actuador Z a posición absoluta
    # print("Moviendo actuador Z a posición 500...")
    # taping_fuyus.move_z_absolute(500)
    # time.sleep(1)
    
    # # Mover ambos actuadores simultáneamente
    # print("Moviendo ambos actuadores a posiciones específicas...")
    # taping_fuyus.move_to_position(500, 250)
    # time.sleep(1)
    
    # # Probar movimiento con control de velocidad
    # print("Moviendo actuador Y con control de velocidad...")
    # taping_fuyus.move_y_with_speed(2000, 1500)  # posición 2000, velocidad 1500
    # time.sleep(1)
    
    # print("Moviendo actuador Z con control de velocidad...")
    # taping_fuyus.move_z_with_speed(1000, 1200)  # posición 1000, velocidad 1200
    # time.sleep(1)
    
    # # Leer contadores después del movimiento
    # print("Leyendo contadores después del movimiento...")
    # counters = taping_fuyus.get_all_counters()
    # print(f"Contadores - Y: {counters['y']}, Z: {counters['z']}")
    
    # # Resetear contadores
    # print("Reseteando contadores...")
    # taping_fuyus.reset_all_counters()
    # time.sleep(0.5)
    
    # # Verificar que los contadores se resetearon
    # print("Verificando reset de contadores...")
    # counters_after_reset = taping_fuyus.get_all_counters()
    # print(f"Contadores después del reset - Y: {counters_after_reset['y']}, Z: {counters_after_reset['z']}")
    
    # # Regresar a home
    # print("Regresando a posición home...")
    # taping_fuyus.home_all_actuators()
    # time.sleep(2)
    
    # print("=== Prueba de TapingFuyus completada ===")


def stampertest():
    """
    Función de prueba para el stamper
    """
    global stamper
    
    if stamper is None:
        print("Error: stamper no está inicializado")
        return False
    
    try:
        print("=== Iniciando pruebas del Stamper ===")
        
        # Prueba de conectividad
        print("1. Probando conectividad...")
        if not stamper.heartbeat():
            print("   ❌ Error: No hay respuesta del stamper")
            return False
        print("   ✅ Conectividad OK")
        
        # Prueba de home
        print("2. Ejecutando home...")
        if stamper.home():
            print("   ✅ Home completado")
        else:
            print("   ❌ Error en home")
            return False
        
        # Prueba de movimiento hacia abajo
        print("3. Moviendo hacia abajo...")
        if stamper.move_down():
            print("   ✅ Movimiento hacia abajo completado")
        else:
            print("   ❌ Error en movimiento hacia abajo")
            return False
        
        # Esperar un poco
        time.sleep(2)
        
        # Prueba de movimiento hacia arriba
        print("4. Moviendo hacia arriba...")
        if stamper.move_up():
            print("   ✅ Movimiento hacia arriba completado")
        else:
            print("   ❌ Error en movimiento hacia arriba")
            return False
        
        print("=== Pruebas del Stamper completadas exitosamente ===")
        return True
        
    except Exception as e:
        print(f"❌ Error durante las pruebas del stamper: {e}")
        return False


def taping_test():
    """
    Función de prueba completa para el firmware taping
    Prueba todos los comandos CAN disponibles (0x01-0x12)
    """
    global taping
    
    if taping is None:
        print("Error: taping no está inicializado")
        return False
    
    try:
        print("=== Iniciando pruebas del Taping ===")
        
        # Prueba 1: Conectividad (Heartbeat - 0x02)
        print("1. Probando conectividad (Heartbeat)...")
        if not taping.heartbeat():
            print("   ❌ Error: No hay respuesta del dispositivo taping")
            return False
        print("   ✅ Conectividad OK")
        
        # Prueba 2: Reset del microcontrolador (0x01)
        # print("2. Probando reset del microcontrolador...")
        # if taping.reset():
        #     print("   ✅ Reset enviado correctamente")
        #     time.sleep(2)  # Esperar a que el dispositivo se reinicie
        # else:
        #     print("   ❌ Error en reset")
        #     return False
        
        # Verificar conectividad después del reset
        # print("   Verificando conectividad después del reset...")
        # time.sleep(1)
        # if not taping.heartbeat():
        #     print("   ❌ Error: No hay respuesta después del reset")
        #     return False
        # print("   ✅ Dispositivo respondiendo después del reset")
        
        # Prueba 3: Comandos individuales de step (0x03-0x0F)
        # print("3. Probando comandos individuales de step...")
        
        # step_commands = [
        #     ("step1 (Feeder)", taping.step1_feeder),
        #     ("step2 (Cutter)", taping.step2_cutter),
        #     ("step3 (Wrapper)", taping.step3_wrapper),
        #     ("step4 (Wrapper Continue)", taping.step4_wrapper_continue),
        #     ("step5 (Wrapper Finish)", taping.step5_wrapper_finish),
        #     ("step6 (Cutter Up)", taping.step6_cutter_up),
        #     ("step7 (Cutter Down)", taping.step7_cutter_down),
        #     ("step8 (Gripper Hold)", taping.step8_gripper_hold),
        #     ("step9 (Gripper Open)", taping.step9_gripper_open),
        #     ("step10 (Gripper Attach)", taping.step10_gripper_attach),
        #     ("step11 (Feeder Reverse)", taping.step11_feeder_reverse),
        #     ("step12 (Feeder Forward)", taping.step12_feeder_forward),
        #     ("step13 (Feeder Reverse Alt)", taping.step13_feeder_reverse_alt)
        # ]
        
        # for step_name, step_function in step_commands:
        #     print(f"   Ejecutando {step_name}...")
        #     if step_function():
        #         print(f"   ✅ {step_name} completado")
        #     else:
        #         print(f"   ❌ Error en {step_name}")
        #         return False
        #     time.sleep(1)  # Pausa entre comandos
        
        # Prueba 4: Secuencias completas (0x10-0x12)
        # print("4. Probando secuencias completas...")
        
        # FullCycle (0x10)
        # print("   Ejecutando FullCycle...")
        # if taping.full_cycle():
        #     print("   ✅ FullCycle completado")
        # else:
        #     print("   ❌ Error en FullCycle")
        #     return False
        # time.sleep(2)
        
        # Forward (0x11)
        # print("   Ejecutando Forward...")
        # if taping.forward():
        #     print("   ✅ Forward completado")
        # else:
        #     print("   ❌ Error en Forward")
        #     return False
        # time.sleep(2)
        
        # Backward (0x12)
        # print("   Ejecutando Backward...")
        # if taping.backward():
        #     print("   ✅ Backward completado")
        # else:
        #     print("   ❌ Error en Backward")
        #     return False
        # time.sleep(2)
        
        # # Prueba 5: Métodos de conveniencia
        # print("5. Probando métodos de conveniencia...")
        
        # print("   Ejecutando feed_tape...")
        # if taping.feed_tape():
        #     print("   ✅ feed_tape completado")
        # else:
        #     print("   ❌ Error en feed_tape")
        #     return False
        # time.sleep(1)
        
        # print("   Ejecutando cut_tape...")
        # if taping.cut_tape():
        #     print("   ✅ cut_tape completado")
        # else:
        #     print("   ❌ Error en cut_tape")
        #     return False
        # time.sleep(1)
        
        # print("   Ejecutando wrap_tape...")
        # if taping.wrap_tape():
        #     print("   ✅ wrap_tape completado")
        # else:
        #     print("   ❌ Error en wrap_tape")
        #     return False
        # time.sleep(1)
        
        # Prueba 6: Operaciones de alto nivel
        # print("6. Probando operaciones de alto nivel...")
        
        # print("   Ejecutando prepare_for_hose...")
        # if taping.prepare_for_hose():
        #     print("   ✅ prepare_for_hose completado")
        # else:
        #     print("   ❌ Error en prepare_for_hose")
        #     return False
        # time.sleep(2)
        
        # print("   Ejecutando complete_taping_cycle...")
        # if taping.complete_taping_cycle():
        #     print("   ✅ complete_taping_cycle completado")
        # else:
        #     print("   ❌ Error en complete_taping_cycle")
        #     return False
        # time.sleep(2)
        
        # print("   Ejecutando home_all...")
        # if taping.home_all():
        #     print("   ✅ home_all completado")
        # else:
        #     print("   ❌ Error en home_all")
        #     return False
        # time.sleep(2)
        
        # Prueba final: Heartbeat para verificar que el dispositivo sigue respondiendo
        # print("7. Verificación final de conectividad...")
        # if taping.heartbeat():
        #     print("   ✅ Dispositivo respondiendo correctamente")
        # else:
        #     print("   ❌ Error: Dispositivo no responde después de las pruebas")
        #     return False
        
        # print("=== Pruebas del Taping completadas exitosamente ===")
        # print("Resumen:")
        # print("✅ Conectividad y reset")
        # print("✅ 13 comandos individuales de step")
        # print("✅ 3 secuencias completas (FullCycle, Forward, Backward)")
        # print("✅ Métodos de conveniencia")
        # print("✅ Operaciones de alto nivel")
        # print("✅ Verificación final")
        
        return True
        
    except Exception as e:
        print(f"❌ Error durante las pruebas del taping: {e}")
        return False


if __name__ == "__main__":
   
    my_main()

    # insertionServosOpen()

    # result = moveElevatorIn()
    # print(f"moveElevatorIn result: {result}")

    # result = movePickandPlace(1)
    # print(f"movePickandPlace result: {result}")
    # #
    # result = oneCycle()
    # print(f"oneCycle result: {result}")
    # #
    # result = pickUpHose()
    # print(f"pickUpHose result: {result}")
    # result = insertionRoutine()
    # print(f"insertionRoutine result: {result}")

    # result = moveTransporter()
    # print(f"moveTransporter result: {result}")
    # result = moveHosepuller()
    # print(f"moveHosepuller result: {result}")
    
    # testHome()

    # insertionServosOpen()

    # result = lubrication_test()
    # print(f"lubrication_test result: {result}")

    # alignCassette()

    # testIR()

    # Prueba de TapingFuyus
    # taping_fuyus_test()

    # Prueba de Taping
    # taping_test()

    # stampertest()


    canbus.close_canbus()


