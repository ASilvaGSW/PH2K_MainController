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
from classes.hose_jig_v2 import HoseJigV2
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
from classes.pick_and_place_camera import PickAndPlaceCamera

# Variables globales para acceder a los objetos desde cualquier función
canbus = None
hose_jig = None
hose_jig_v2 = None
hose_puller = None
puller_extension = None
insertion_jig = None
elevator_in = None
pick_and_place = None
pick_and_place_camera = None
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
CHECK_SECOND_STATION = False
CHECK_ALL_DEVICES = True  # When True, checks all devices regardless of station flags

#Initialize and setup CANbus and devices
def my_main(check):
    global canbus, hose_jig, hose_jig_v2, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos, pick_and_place_camera
    global lubrication_feeder, lubrication_stamper, stamper, taping, taping_fuyus, transporter_fuyus, transporter_grippers
    
    # Example CAN IDs (these should be configured according to your hardware setup)
    CANBUS_ID_JIG = 0x0CA
    CANBUS_ID_HOSE_JIG_V2 = 0x0CB  # Adjust if needed per hardware mapping
    CANBUS_ID_PULLER = 0x192
    CANBUS_ID_EXTENSION = 0x193
    CANBUS_ID_INSERTION = 0x0C9
    CANBUS_ID_ELEVATOR_IN = 0x189   
    CANBUS_ID_PICK_AND_PLACE = 0x191
    CANBUS_ID_PICK_AND_PLACE_CAMERA = 0x001
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
    hose_jig_v2 = HoseJigV2(canbus, CANBUS_ID_HOSE_JIG_V2)
    hose_puller = HosePuller(canbus, CANBUS_ID_PULLER)
    puller_extension = PullerExtension(canbus, CANBUS_ID_EXTENSION)
    insertion_jig = InsertionJig(canbus, CANBUS_ID_INSERTION)
    elevator_in = ElevatorIn(canbus, CANBUS_ID_ELEVATOR_IN)
    pick_and_place = PickAndPlace(canbus, CANBUS_ID_PICK_AND_PLACE)
    pick_and_place_camera = PickAndPlaceCamera(canbus, CANBUS_ID_PICK_AND_PLACE_CAMERA)
    insertion_servos = InsertionServos(canbus, CANBUS_ID_INSERTION_SERVOS)
    lubrication_feeder = LubricationFeeder(canbus, CANBUS_ID_LUBRICATION_FEEDER)
    lubrication_stamper = LubricationStamper(canbus, CANBUS_ID_LUBRICATION_STAMPER)
    stamper = Stamper(canbus, CANBUS_ID_STAMPER)
    taping = Taping(canbus, CANBUS_ID_TAPING)
    taping_fuyus = TapingFuyus(canbus, CANBUS_ID_TAPING_FUYUS)
    transporter_fuyus = TransporterFuyus(canbus, CANBUS_ID_TRANSPORTER_FUYUS)
    transporter_grippers = TransporterGrippers(canbus, CANBUS_ID_TRANSPORTER_GRIPPERS)

    if check:
     checkConnectivity()

#Check connectivity
def checkConnectivity():
    
    global canbus, hose_jig, hose_jig_v2, hose_puller, puller_extension, insertion_jig, elevator_in, pick_and_place, insertion_servos
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
        # print("Hose Jig V2 Connected") if hose_jig_v2.ping() == "success" else print("Hose Jig V2 Not Connected")
        # print("Lubrication Stamper Connected") if lubrication_stamper.send_heartbeat() == "success" else print("Lubrication Stamper Not Connected")
        # print("Stamper Connected") if stamper.send_heartbeat() == "success" else print("Stamper Not Connected")
        # print("Taping Connected") if taping.send_heartbeat() == "success" else print("Taping Not Connected")
        # print("Taping Fuyus Connected") if taping_fuyus.send_heartbeat() == "success" else print("Taping Fuyus Not Connected")
        print("Transporter Fuyus Connected") if transporter_fuyus.send_heartbeat() == "success" else print("Transporter Fuyus Not Connected")
        print("Transporter Grippers Connected") if transporter_grippers.send_heartbeat() == "success" else print("Transporter Grippers Not Connected")

    # Show current connectivity check configuration
    if not CHECK_ALL_DEVICES:
        print(f"\n=== CONNECTIVITY CHECK CONFIG ===")
        print(f"First Station: {'ENABLED' if CHECK_FIRST_STATION else 'DISABLED'}")
        print(f"Second Station: {'ENABLED' if CHECK_SECOND_STATION else 'DISABLED'}")
    else:
        print(f"\n=== CONNECTIVITY CHECK CONFIG ===")
        print("Checking ALL DEVICES (station flags ignored)")

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


#****************** Core Functions ***********************

# ReceiveMaterial
def ReceiveMaterial():
    global elevator_in

    home = [0, 0, 0]
    safety_high = -300
    cassette_deliver_high = -608
    transportation_high = -500

    # left
    pick_left = [-225, -315, -820]
    deliver_left = [-225, -1800, -500]

    # Right
    pick_right = [-1358, -317, -850]
    deliver_right = [-1353, -1800, -500]

    # Homing
    if elevator_in.home_elevator_z() != "success": return "error04"

    if elevator_in.move_gantry_z(transportation_high) != "success": return "error08"
    if elevator_in.move_gantry_array(deliver_left) != "success": return "error09"

    input("Continue?")

    if elevator_in.move_gantry_z(safety_high) != "success": return "error20"
    if elevator_in.move_gantry_array(home) != "success": return "error21"


    return "success"


#Move Elevator In
def moveElevatorIn():

    global elevator_in,pick_and_place

    home = [0,0,0]
    safety_high = -300
    cassette_deliver_high = -600
    transportation_high = -500

    #left
    pick_left = [-225,-319,-815]
    pick_left = [-225,-314,-815]
    deliver_left = [-250,-1800,-500]
    
    #Right
    pick_right = [-1358,-319,-835]
    pick_right = [-1355,-319,-835]
    deliver_right = [-1365,-1800,-500]
    deliver_right2 = [-1410,-1800,-500]

    # if elevator_in.move_elevator_z(-5000) != "success": return "error05"
    # return ReceiveMaterial()

    # #Homing
    # if elevator_in.home_gantry_z() != "success": return "error01"
    # if elevator_in.home_gantry_x() != "success": return "error02"
    # if elevator_in.home_gantry_y() != "success": return "error03"
    # if elevator_in.home_elevator_z() != "success": return "error04"

    # return ""

    #Aligning Cassette
    if elevator_in.move_elevator_until_sensor(0, 200) != "success": return "error05"


    #Left Side
    if elevator_in.move_gantry_array(pick_left) != "success": return "error06"

    # return ""

    if elevator_in.close_gripper() != "success": return "error07"
    time.sleep(.25)
    if elevator_in.move_gantry_z(transportation_high) != "success": return "error08"
    if elevator_in.move_gantry_array(deliver_left) != "success": return "error09"
    if elevator_in.move_gantry_z(cassette_deliver_high+10) != "success": return "error10"
    if elevator_in.open_gripper() != "success": return "error11"
    time.sleep(.25)
    if elevator_in.move_gantry_z(safety_high) != "success": return "error12"
    if elevator_in.move_gantry_array(home) != "success": return "error13"


    #Right Side
    if elevator_in.move_gantry_array(pick_right) != "success": return "error14"
    if elevator_in.close_gripper() != "success": return "error15"
    time.sleep(.25)
    if elevator_in.move_gantry_z(transportation_high) != "success": return "error16"
    if elevator_in.move_gantry_array(deliver_right) != "success": return "error17"
    if elevator_in.move_gantry_array(deliver_right2) != "success": return "error17"
    if elevator_in.move_gantry_z(cassette_deliver_high) != "success": return "error18"
    if elevator_in.open_gripper() != "success": return "error19"
    time.sleep(.25)
    if elevator_in.move_gantry_z(safety_high) != "success": return "error20"
    if elevator_in.move_gantry_array(home) != "success": return "error21"

    #conveyors
    if pick_and_place.start_left_conveyor() != "success": return "error22"
    if pick_and_place.start_right_conveyor() != "success": return "error23"
    time.sleep(3)
    if pick_and_place.stop_left_conveyor() != "success": return "error24"
    if pick_and_place.stop_right_conveyor() != "success": return "error25"

    # alignCassette()

    return "success"


#Align Cassette:
def alignCassette():
    global pick_and_place,pick_and_place_camera

    if pick_and_place.move_left_conveyor_until_sensor(0,1000) != "success" : return "error01"
    if pick_and_place.move_right_conveyor_until_sensor(0,1000) != "success" : return "error02"

    time.sleep(1)

    # alignComponent()

def alignComponent():
    pick_and_place_camera.alignment_joint()
    pick_and_place_camera.alignment_nozzle()

#Move Pick and Place
def movePickandPlace(n=1,need=False):
    global pick_and_place, insertion_servos, insertion_jig

    receiving_x = 6500
    receiving_z = 7000
    translation_gap = 0.73

    # Initial Setup
    if insertion_servos.slider_joint_receive() != "success": return "error03"
    if insertion_servos.slider_nozzle_receive() != "success": return "error3.1"
    if insertion_jig.move_z_axis(receiving_z) != "success": return "error04"
    if insertion_jig.move_x_axis(receiving_x) != "success": return "error05"
    if pick_and_place.open_gripper() != "success": return "error06"

    gap = 0
    zgap = 0

    #Nozzle Data
    nozzle_high = -965
    nozzle_x = [-580,-450,-300,-170,-20,110]
    trans_nozzle_high = -600

    # deliver_nozzle_x = -3697
    deliver_nozzle_x = -3633
    deliver_nozzle_z = -1005

    # Joint Data
    # joint_high = -1243
    joint_high = -965
    joint_x = [-1770,-1690,-1610,-1530,-1450,-1370,-1290,-1210,-1130,-1050]
    trans_joint_high = -600

    # deliver_joint_x = -3990
    deliver_joint_x = -3919
    deliver_joint_z = -1005

    # Validate Home

    if pick_and_place.move_actuator_z(0) != "success": return "error07"
    if pick_and_place.move_actuator_x(0) != "success": return "error08"
    #
    # # Pick Nozzle
    if pick_and_place.move_actuator_x(nozzle_x[n]- gap) != "success": return "error09"


    if need:
        if pick_and_place.move_actuator_z(nozzle_high + 100) != "success": return "error10"
        input("continue?")

    if pick_and_place.move_actuator_z(nozzle_high+zgap) != "success": return "error10"
    #



    if pick_and_place.close_gripper() != "success": return "error11"
    # #
    # # Deliver Nozzle
    if pick_and_place.move_actuator_z(trans_nozzle_high) != "success": return "error12"
    if pick_and_place.move_actuator_x(deliver_nozzle_x - gap) != "success": return "error13"

    if pick_and_place.move_actuator_z(deliver_nozzle_z+zgap) != "success": return "error14"

    if pick_and_place.open_gripper() != "success": return "error15"
    time.sleep(.5)


    # Back to Home
    # if pick_and_place.move_actuator_z(0,False) != "success": return "error16"
    # if pick_and_place.move_actuator_x(0,False) != "success": return "error17"

    # return ""

    # *******************************************************

    # Joint Pick Up

    # Home (ensure is there before moving)
    if pick_and_place.move_actuator_z(0) != "success": return "error18"
    # if pick_and_place.move_actuator_x(0) != "success": return "error19"

    # Pick Joint
    if pick_and_place.move_actuator_x(joint_x[n]-gap) != "success": return "error21"

    if need:
        if pick_and_place.move_actuator_z(joint_high + 100) != "success": return "error22"
        input("continue?")

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

    lubrication_position_z = 380 + offset_z
    lubricate_nozzle = 1400 + offset_x

    insertion_position_z = 380 + offset_z
    insert_nozzle = 2890 + offset_x

    librication_position_joint_z = 380 + offset_z
    lubricate_joint = 7420 + offset_x

    insertion_position_joint_z = 380 + offset_z
    insert_joint = 5240 + offset_x

    #****************************** Hose Puller Data ******************************

    safe_position = 200
    safe_position_over_hose_jig = 242
    home_y = 4210
    wait_y = 5930
    cutting_position = 7830
    pickup_y = 9015
    before_rise_position = 8510
    z_home = 50
    z_picking_position = 80
    alignmnet_for_joint = 4860
    alignmnet_for_joint = 5000

    #****************************** Custom Variables ******************************

    insertion_jig_safe_zone = 4000
    preefeder_speed = 50
    feed_hose_time = 3.1
    lubricate_nozzle_time = 0.15
    lubricate_joint_time = 0.08
    hose_puller_y_speed = 200
    hose_puller_y_speed_for_alignment = 20

    #****************************** Routine ******************************

    if hose_jig.gripper_open() != "success" : return "error01"

    # if insertion_servos.activate_cutter() != "success" : return "error02"
    if insertion_servos.slider_nozzle_receive() != "success" : return "error02"

    # Feed Hose

    if lubrication_feeder.close_hose_holder() != "success" : return "error03"
    if lubrication_feeder.feed_hose(duration=feed_hose_time,speed=520) != "success" : return "error04"
    # if insertion_servos.activate_cutter() != "success": return "error02"

    # return ""

    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error05"
    if lubrication_feeder.open_hose_holder() != "success" : return "error06"

    # Clamping and Reset Gripper
    if insertion_servos.clamp_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_joint_close() != "success" : return "error08"
    if puller_extension.open_gripper() != "success" : return "error09"

    # Homing Slider Before Movement
    if insertion_servos.slider_joint_home() != "success" : return "error10"
    if insertion_servos.slider_nozzle_receive() != "success" : return "error11"

    # Insertio Jig Home POsition

    if hose_jig.insertion_position(False) != "success" : return "error12"
    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error13"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error14"


    #Lubricate Hose

    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error15"
    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error16"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error17"
    if lubrication_feeder.lubricate_nozzle(lubricate_nozzle_time) != "success" : return "error18"

    # Nozzle Insertion
    insertion_jig.move_z_axis(insertion_position_z)
    if insertion_jig.move_x_axis(insert_nozzle) != "success" : return "error20"
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error21"
    if insertion_servos.clamp_nozzle_close() != "success" : return "error22"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success" : return "error23"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success" : return "error24"
    if insertion_servos.clamp_nozzle_open() != "success" : return "error25"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_receive() != "success" : return "error26"

    # Go to Down Position for Hose Puller
    if insertion_jig.move_z_axis(insertion_jig_safe_zone) != "success" : return "error27"
    #
    # # Starting Prefeeder
    if lubrication_feeder.move_pre_feeder(preefeder_speed) != "success" : return "error28"

    # Preparing Hose Puller and Hose Jig
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error29"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error30"
    if hose_jig.insertion_position() != "success" : return "error31"

    # Pulling Action
    if hose_puller.move_y_actuator(pickup_y) != "success" : return "error32"
    if hose_puller.move_z_actuator(z_picking_position) != "success" : return "error33"
    if puller_extension.close_gripper() != "success" : return "error34"
    if hose_puller.move_y_actuator_with_speed(before_rise_position,hose_puller_y_speed) != "success" : return "error35"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error36"
    if hose_puller.move_y_actuator_with_speed(cutting_position,hose_puller_y_speed) != "success" : return "error37"
    if hose_puller.move_z_actuator(safe_position_over_hose_jig) != "success" : return "error38"
    if hose_puller.move_y_actuator_with_speed(wait_y,hose_puller_y_speed) != "success" : return "error39"

    # Cutting Hose
    if lubrication_feeder.close_hose_holder() != "success" : return "error40"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success" : return "error41"
    time.sleep(.5)
    # if lubrication_feeder.open_hose_holder() != "success" : return "error40"


    # Alignment for Joint Insertion
    if hose_puller.move_y_actuator_with_speed(alignmnet_for_joint,200) != "success" : return "error13" #Aqui hare el cambio
    if insertion_servos.holder_hose_joint_semi_close() != "success" : return "error42"
    time.sleep(1)
    if hose_puller.move_y_axis_until_no_hose(hose_puller_y_speed_for_alignment) != "success" : return "error43"
    # if hose_puller.move_y_actuator_relative_with_speed(50) != "success" : return "error13.1"

    # Stoping Prefeeder
    if lubrication_feeder.move_pre_feeder(0) != "success" : return "error44"

    # Lubricate Hose on Joint Area
    if insertion_servos.holder_hose_joint_close() != "success" : return "error45"
    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error46"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error47"
    if lubrication_feeder.lubricate_joint(lubricate_joint_time) != "success" : return "error48"
    time.sleep(0.5)

    # Joint Insertion
    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error49"
    insertion_jig.move_z_axis(insertion_position_joint_z)
    if insertion_servos.clamp_joint_close() != "success" : return "error51"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success" : return "error52"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success" : return "error53"
    if insertion_servos.clamp_joint_open() != "success" : return "error54"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success" : return "error55"
    time.sleep(0.2)

    # Finish Pulling Action

    if hose_puller.move_y_actuator(home_y) != "success" : return "error56"

    # Homing for Finish
    if insertion_jig.move_z_axis(insertion_jig_safe_zone,False) != "success" : return "error57"
    if insertion_jig.move_x_axis(0,False) != "success" : return "error58"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error59"

    # Delivering Hose

    if hose_jig.gripper_close() != "success" : return "error60"
    if puller_extension.open_gripper() != "success" : return "error61"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error62"
    if hose_jig.deliver_position() != "success" : return "error63"
    if hose_jig.gripper_open() != "success" : return "error64"
    if hose_puller.move_z_actuator(0) != "success" : return "error65"

    return "success"


#Pick Hose
def pickUpHoseFrom1stStation():

    global transporter_fuyus, transporter_grippers, hose_jig,hose_jig_v2

    # transporter_fuyus.home_x_axis()
    # transporter_fuyus.home_gripper1()
    # transporter_fuyus.home_gripper2()
    #
    # return ""


    if hose_jig.deliver_position() != "success" : return "001"
    if hose_jig.gripper_close() != "success" : return "001"
    if transporter_fuyus.home_gripper1() != "success" : return "01"
    if transporter_fuyus.home_gripper2() != "success" : return "01"

    if transporter_fuyus.moveToReceivingPosition() != "success" : return "03"

    if transporter_fuyus.pickHoseFromFirstStation() != "success" : return "04"

    if transporter_grippers.close_all_grippers() != "success" : return "05"

    time.sleep(1)

    if hose_jig.gripper_open() != "success" : return "06"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToDeliverPosition() != "success": return "03"

    # if transporter_fuyus.stamperHigh() != "success": return "04"

    if transporter_grippers.open_all_grippers() != "success": return "05"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToSafeSpace() != "success": return "08"


    return "success"

def pickUpHoseFromStamper():

    global transporter_fuyus, transporter_grippers, hose_jig_v2

    if hose_jig_v2.close_stamper_hose_jig() != "success": return "06"

    if transporter_fuyus.pickHome() != "success" : return "01"

    if transporter_fuyus.moveToStamperHoseJig() != "success" : return "02"
    if transporter_fuyus.stamperHigh() != "success" : return "04"
    if transporter_grippers.close_all_grippers() != "success" : return "05"

    time.sleep(1)

    if hose_jig_v2.open_stamper_hose_jig() != "success" : return "06"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToTapingHoseJig() != "success": return "03"

    if transporter_fuyus.tapingHigh() != "success": return "04"

    if transporter_grippers.open_all_grippers() != "success": return "05"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToSafeSpace() != "success": return "08"

    return "success"

def pickUpHoseFromTaping():
    global transporter_fuyus, transporter_grippers, hose_jig_v2

    if hose_jig_v2.close_taping_hose_jig() != "success": return "06"

    if transporter_fuyus.pickHome() != "success" : return "01"

    if transporter_fuyus.moveToTapingHoseJig() != "success" : return "02"
    if transporter_fuyus.tapingHigh() != "success" : return "04"
    if transporter_grippers.close_all_grippers() != "success" : return "05"

    time.sleep(1)

    if hose_jig_v2.open_taping_hose_jig() != "success" : return "06"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToDeliverPosition() != "success": return "03"

    if transporter_grippers.open_all_grippers() != "success": return "05"

    if transporter_fuyus.moveToSafeSpace() != "success": return "08"

    return "success"


#Test all home functions
def testHome():
    # """Test function for new go home methods"""
    global elevator_in, pick_and_place, hose_puller, hose_jig, insertion_jig, insertion_servos,transporter_fuyus

    insertionServosOpen()
    insertion_servos.slider_joint_home()


    print("Testing new go home functions...")

    # Test ElevatorIn homing functions
    print("\n--- Testing ElevatorIn homing functions ---")

    print("Testing home_elevator_z...")
    result = elevator_in.home_elevator_z()
    print(f"Result: {result}")

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


    # Test PickAndPlace homing functions
    print("\n--- Testing PickAndPlace homing functions ---")

    result = pick_and_place.home_z_axis()
    print(f"Result: {result}")

    print("Testing home_x_axis...")
    result = pick_and_place.home_x_axis()
    print(f"Result: {result}")

    print("Testing home_z_axis...")


    # Test HoseJig homing function
    print("\n--- Testing HoseJig homing function ---")

    print("Testing home_actuator...")
    result = hose_jig.go_home()
    print(f"Result: {result}")

    # Test HosePuller homing functions
    print("\n--- Testing HosePuller homing functions ---")

    print("Testing home_y_axis...")
    result = hose_puller.home_y_axis()
    print(f"Result: {result}")

    print("Testing home_z_axis...")
    result = hose_puller.home_z_axis()
    print(f"Result: {result}")


    # Test InsertionJig homing functions
    print("\n--- Testing InsertionJig homing functions ---")

    print("Testing home_x_axis...")
    result = insertion_jig.home_x_axis_go_home()
    print(f"Result: {result}")

    print("Testing home_z_axis...")
    result = insertion_jig.home_z_axis_go_home()
    print(f"Result: {result}")

    # # Enviar Z a Home
    # print("Enviando Z a Home...")
    # taping_fuyus.home_z_actuator()
    #
    # # Enviar Y a Home
    # print("Enviando Y a Home...")
    # taping_fuyus.home_y_actuator()
    #
    # taping_fuyus.move_y_actuator_with_speed(10000, 400)


    # Test Transporter homing functions
    print("\n--- Testing Transporter homing functions ---")

    print("Testing home_x_axis...")
    result = transporter_fuyus.home_x_axis()
    print(f"Result: {result}")

    print("\nAll homing tests completed!")

    return "success"


#Test lubrication
def lubrication_test():
    global lubrication_feeder, insertion_servos

    if lubrication_feeder.lubricate_nozzle(duration=5) != "success": return "error01"
    if lubrication_feeder.lubricate_joint(duration=5) != "success": return "error02"

    return "success"


def tape():
    global taping_fuyus, hose_jig_v2, taping


    home_position = 10000
    tape_position = 830
    z_speed = 150
    tape_spot = 6980
    y_speed = 400

    # # Enviar Z a Home
    # print("Enviando Z a Home...")
    # taping_fuyus.home_z_actuator()
    # time.sleep(0.5)
    #
    # # Enviar Y a Home
    # print("Enviando Y a Home...")
    # taping_fuyus.home_y_actuator()
    # time.sleep(0.5)
    #
    taping_fuyus.move_y_actuator_with_speed(home_position, y_speed)

    # taping_fuyus.move_z_actuator_with_speed(0, z_speed)
    # taping_fuyus.move_y_actuator_with_speed(tape_spot, y_speed)


    #Start Feeding Routine
    taping.execute_forward_sequence()

    time.sleep(5)

    # Start Feeding Routine
    hose_jig_v2.close_taping_hose_jig()

    # Go to Tape Position
    taping_fuyus.move_z_actuator_with_speed(0, z_speed)
    taping_fuyus.move_y_actuator_with_speed(tape_spot, y_speed)

    # Get Lower
    taping_fuyus.move_z_actuator_with_speed(tape_position - 80, z_speed)

    # Tape Preparation

    taping.open_gripper()
    taping.cut_tape()
    time.sleep(3)

    # Get to Tape Position
    taping_fuyus.move_z_actuator_with_speed(tape_position, z_speed)

    time.sleep(1)

    # Wrapping Routine
    taping.execute_backward_sequence()

    time.sleep(8)

    #Returning Home

    taping_fuyus.move_z_actuator_with_speed(0, z_speed)
    taping_fuyus.move_y_actuator_with_speed(home_position, y_speed)

    #Open Hose Jig
    hose_jig_v2.open_taping_hose_jig()

    return "Success"


# ******************************** Only Insertion Routine ********************************
def insertionRoutine():

    global insertion_jig, insertion_servos, lubrication_feeder

    #****************************** Insertion Jig ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4200 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 380 + offset_z
    lubricate_nozzle = 1400 + offset_x

    insertion_position_z = 380 + offset_z
    insert_nozzle = 2890 + offset_x

    librication_position_joint_z = 380 + offset_z
    lubricate_joint = 7420 + offset_x

    insertion_position_joint_z = 380 + offset_z
    insert_joint = 5240 + offset_x

    # #****************************** Routine ******************************

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

    if lubrication_feeder.lubricate_nozzle(.5) != "success": return "error19"

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

    input("Continue?")

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

    if insertion_jig.move_z_axis(home_position_z) != "success": return "error41"
    if insertion_jig.move_x_axis(home_position_x) != "success": return "error42"




    return "success"

def taping():
    """
    Función de prueba para la clase TapingFuyus
    Prueba las funciones principales de control de actuadores Y y Z
    """
    global taping_fuyus, hose_jig_v2, taping

    print("=== Iniciando prueba de TapingFuyus ===")

    # Enviar heartbeat
    # print("Enviando heartbeat...")
    # taping_fuyus.send_heartbeat()
    # time.sleep(0.5)

    home_position = 10000
    tape_position = 830
    z_speed = 150
    tape_spot = 1640
    y_speed = 400


    while True:

        print("==============================")
        print("Seleccione una opción / Select an option / オプションを選択")
        print("1: Enviar Z y Y a Home / Home Z & Y / ZとYホーム")
        print("2: Mover servos a 7 y 92 / Move servos 7 & 92 / サーボ7と92")
        print("3: Verificar alineación / Check alignment / アライメント確認")
        print("4: Secuencia adelante con corte y regreso / Forward seq with cut / 前進シーケンス+カット")
        print("5: Volver ambos actuadores a Home y servos 7 / Return Home / ホームに戻す")
        print("==============================")

        try:
            x = int(input("Ingrese opción (1-5) / Enter option (1-5) / オプション(1-5): "))
        except ValueError:
            print("Entrada inválida")
            continue

        if x == 1:

            # Enviar Z a Home
            print("Enviando Z a Home...")
            taping_fuyus.home_z_actuator()
            time.sleep(0.5)

            # Enviar Y a Home
            print("Enviando Y a Home...")
            taping_fuyus.home_y_actuator()
            time.sleep(0.5)

        elif x == 2:

            hose_jig_v2.move_servos_absolute(7, 2)
            hose_jig_v2.move_servos_absolute(92, 2)

        elif x == 3:

            taping.checkAlignment()

        elif x == 4:

            taping.execute_forward_sequence()

            time.sleep(5)

            hose_jig_v2.move_servos_absolute(92, 2)

            taping_fuyus.move_z_actuator_with_speed(0, z_speed)
            taping_fuyus.move_y_actuator_with_speed(6590, y_speed)

            taping_fuyus.move_z_actuator_with_speed(tape_position - 80, z_speed)

            taping.open_gripper()
            taping.cut_tape()
            time.sleep(3)

            taping_fuyus.move_z_actuator_with_speed(tape_position, z_speed)

            time.sleep(1)

            taping.execute_backward_sequence()

        elif x == 5:

            taping_fuyus.move_z_actuator_with_speed(0, z_speed)
            taping_fuyus.move_y_actuator_with_speed(home_position, y_speed)

            hose_jig_v2.move_servos_absolute(7, 2)
        else:
            print("Opción no válida")

    return True


# ******************************** Helper functions to control connectivity flags ********************************

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


# ******************************** Taping / Stamping Debugging Functions ********************************


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


def testFeed():
    global lubrication_feeder,insertion_servos

    # Feed Hose
    if lubrication_feeder.close_hose_holder() != "success": return "error03"
    if lubrication_feeder.feed_hose(duration=3.1, speed=520) != "success": return "error04"
    time.sleep(1)
    if insertion_servos.activate_cutter() != "success": return "error02"
    if lubrication_feeder.open_hose_holder() != "success": return "error06"

if __name__ == "__main__":

    my_main(False)

    # testHome()

    testFeed()

    # moveElevatorIn()
    #
    # alignCassette()

    # alignComponent()

    # input("Continue?")
    #
    # for i in range(0,4):
    # movePickandPlace(3,False)
    #     print(oneCycle())
    #
    # input("Continue?")
    #
    # print(oneCycle())
    #
    # input("Continue?")
    #
    # pickUpHoseFrom1stStation()
    #
    # input("Continue?")
    #
    # pickUpHoseFromStamper()

    # input("Continue?")

    # tape()

    # input("Continue?")

    # pickUpHoseFromTaping()


    canbus.close_canbus()