from flask import Blueprint, request, jsonify, session
from datetime import datetime
import sys
import os
import time

# Add the main_controller directory to Python path
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'main_controller'))

from classes.hose_jig import HoseJig
from classes.hose_puller import HosePuller
from classes.puller_extension import PullerExtension
from classes.insertion_jig import InsertionJig
from classes.elevator_in import ElevatorIn
from classes.elevator_out import ElevatorOut
from classes.pick_and_place import PickAndPlace
from classes.insertion_servos import InsertionServos
from classes.lubrication_feeder import LubricationFeeder
from classes.transporter_fuyus import TransporterFuyus
from classes.transporter_grippers import TransporterGrippers
from classes.pick_and_place_camera import PickAndPlaceCamera

# Now we can import the Canbus class
try:
    if sys.platform.startswith('win32'):
        from classes.canbus import Canbus
        print("Windows detected. Importing Canbus from classes.canbus")
    elif sys.platform.startswith('linux'):
        from classes.canbus_jetson import Canbus
        print("Linux detected. Importing Canbus from classes.canbus_jetson")
    else:
        raise ImportError("Unsupported operating system for CAN bus")

    print("Canbus class found. CAN bus functionality will be enabled.")

    canbus = Canbus()
    start_result = canbus.start_canbus()
    print(f"Initial CAN bus start result: {start_result}")
    print(f"CAN bus is_started: {canbus.is_started}")

except ImportError as e:
    # Fallback if import fails - we'll handle this gracefully
    print(f"Canbus class not found: {e}. CAN bus functionality will be disabled.")
    canbus = None
except Exception as e:
    print(f"Error initializing CAN bus: {e}")
    canbus = None

# Ids
CANBUS_ID_JIG = 0x0CA
CANBUS_ID_PULLER = 0x192
CANBUS_ID_EXTENSION = 0x193
CANBUS_ID_INSERTION = 0x0C9
CANBUS_ID_ELEVATOR_IN = 0x189
CANBUS_ID_ELEVATOR_OUT = 0x188
CANBUS_ID_PICK_AND_PLACE = 0x191
CANBUS_ID_INSERTION_SERVOS = 0x002
CANBUS_ID_LUBRICATION_FEEDER = 0x019
CANBUS_ID_TRANSPORTER_FUYUS = 0x021
CANBUS_ID_TRANSPORTER_GRIPPERS = 0x020
CANBUS_ID_PICK_AND_PLACE_CAMERA = 0x001

#Instance Declaration
hose_jig = HoseJig(canbus, CANBUS_ID_JIG)
hose_puller = HosePuller(canbus, CANBUS_ID_PULLER)
puller_extension = PullerExtension(canbus, CANBUS_ID_EXTENSION)
insertion_jig = InsertionJig(canbus, CANBUS_ID_INSERTION)
elevator_in = ElevatorIn(canbus, CANBUS_ID_ELEVATOR_IN)
elevator_out = ElevatorOut(canbus, CANBUS_ID_ELEVATOR_OUT)
pick_and_place = PickAndPlace(canbus, CANBUS_ID_PICK_AND_PLACE)
insertion_servos = InsertionServos(canbus, CANBUS_ID_INSERTION_SERVOS)
lubrication_feeder = LubricationFeeder(canbus, CANBUS_ID_LUBRICATION_FEEDER)
transporter_fuyus = TransporterFuyus(canbus, CANBUS_ID_TRANSPORTER_FUYUS)
transporter_grippers = TransporterGrippers(canbus, CANBUS_ID_TRANSPORTER_GRIPPERS)
pick_and_place_camera = PickAndPlaceCamera(canbus, CANBUS_ID_PICK_AND_PLACE_CAMERA)

#Move Pick and Place
def movePickandPlace():
    global pick_and_place, insertion_servos, insertion_jig, pick_and_place_camera

    receiving_x = 6500
    receiving_z = 7000

    gap = -10
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

    # Initial Setup
    if insertion_servos.slider_joint_receive() != "success": return "error01"
    if insertion_servos.slider_nozzle_receive() != "success": return "error02"
    if insertion_jig.move_z_axis(receiving_z) != "success": return "error03"
    if insertion_jig.move_x_axis(receiving_x) != "success": return "error04"
    if pick_and_place.open_gripper() != "success": return "error05"

    # Validate Home

    if pick_and_place.move_actuator_z(0) != "success": return "error06"
    if pick_and_place.move_actuator_x(0) != "success": return "error07"
    #
    # Pick Nozzle
    # Get index from camera for Nozzle
    n_nozzle = pick_and_place_camera.pick_up_nozzle()
    print(n_nozzle)

    # Check for empty row (0xFF)
    if n_nozzle == 255:
        print("Nozzle row empty, aligning...")
        pick_and_place_camera.custom_alignment_nozzle()
        first_pick_after_align = True
        # Retry after alignment
        n_nozzle = pick_and_place_camera.pick_up_nozzle()
        if n_nozzle == 255:
            return "error08"

    if n_nozzle is None:
        return "error08.1"

    # Ensure index is within bounds
    if n_nozzle >= len(nozzle_x):
        n_nozzle = len(nozzle_x) - 1 # Fallback or error? defaulting to last valid or erroring out
        # return f"error09_index_{n_nozzle}" # Let's try to proceed or return error. Returning error is safer.
        return f"error09_index_{n_nozzle}"

    if pick_and_place.move_actuator_x(nozzle_x[n_nozzle]- gap) != "success": return "error09"
    if pick_and_place.move_actuator_z(nozzle_high+zgap) != "success": return "error10"
    if pick_and_place.close_gripper() != "success": return "error11"

    # Deliver Nozzle
    if pick_and_place.move_actuator_z(trans_nozzle_high) != "success": return "error12"
    if pick_and_place.move_actuator_x(deliver_nozzle_x - gap) != "success": return "error13"

    if pick_and_place.move_actuator_z(deliver_nozzle_z+zgap) != "success": return "error14"

    if pick_and_place.open_gripper() != "success": return "error15"
    time.sleep(.5)

    # Joint Pick Up

    # Home (ensure is there before moving)
    if pick_and_place.move_actuator_z(0) != "success": return "error16"
    # if pick_and_place.move_actuator_x(0) != "success": return "error19"

    # Pick Joint
    
    # Get index from camera for Joint
    n_joint = pick_and_place_camera.pick_up_joint()
    
    # Check for empty row (0xFF)
    if n_joint == 255:
        print("Joint row empty, aligning...")
        pick_and_place_camera.custom_alignment_joint()
        first_pick_after_align = True
        # Retry after alignment
        n_joint = pick_and_place_camera.pick_up_joint()
        if n_joint == 255:
            return "error17"
            
    if n_joint is None:
        return "error17.1"

    # Ensure index is within bounds
    if n_joint >= len(joint_x):
        return f"error21_index_{n_joint}"

    if pick_and_place.move_actuator_x(joint_x[n_joint]-gap) != "success": return "error18"
    if pick_and_place.move_actuator_z(joint_high+zgap) != "success": return "error20"
    if pick_and_place.close_gripper() != "success": return "error21"

    # Deliver Joint
    if pick_and_place.move_actuator_z(trans_joint_high) != "success": return "error22"
    if pick_and_place.move_actuator_x(deliver_joint_x-gap) != "success": return "error23"
    if pick_and_place.move_actuator_z(deliver_joint_z+zgap) != "success": return "error24"
    if pick_and_place.open_gripper() != "success": return "error25"


    # Go Back to Home
    if pick_and_place.move_actuator_z(0,False) != "success": return "error26"
    if pick_and_place.move_actuator_x(0,False) != "success": return "error27"

    time.sleep(1)

    first_pick_after_align = False

    return "success"

#Main One Cycle
def insertionServosOpen():
    global insertion_servos

    insertion_servos.slider_joint_home()
    insertion_servos.slider_nozzle_home()
    insertion_servos.clamp_nozzle_open()
    insertion_servos.clamp_joint_open()
    insertion_servos.cutter_open()
    insertion_servos.holder_hose_joint_open()
    insertion_servos.holder_hose_nozzle_open()

def oneCycle():

    global insertion_jig, insertion_servos, hose_puller, hose_jig
    global puller_extension,pick_and_place,lubrication_feeder

    #****************************** Insertion Jig Data ******************************

    offset_x = 0
    offset_z = 0

    home_position_z = 4200 + offset_z
    home_position_x = 0 + offset_x

    lubrication_position_z = 350 + offset_z
    lubricate_nozzle = 1330 + offset_x

    insertion_position_z = 330 + offset_z
    insert_nozzle = 2860 + offset_x

    librication_position_joint_z = 350 + offset_z
    lubricate_joint = 7420 + offset_x

    insertion_position_joint_z = 365 + offset_z
    insert_joint = 5210 + offset_x

    #****************************** Hose Puller Data ******************************

    safe_position = 200
    safe_position_over_hose_jig = 242
    home_y = 4210
    wait_y = 5930
    wait_y = 7830
    cutting_position = 7830
    pickup_y = 9015
    before_rise_position = 8510
    z_home = 50
    z_picking_position = 80
    alignmnet_for_joint = 4860
    alignmnet_for_joint = 6800

    #****************************** Custom Variables ******************************

    insertion_jig_safe_zone = 4000
    preefeder_speed = 50
    feed_hose_time = 3.0
    lubricate_nozzle_time = 0.03
    lubricate_joint_time = 0.03
    hose_puller_y_speed = 200
    hose_puller_y_speed_for_alignment = 10
    feeder_speed = 290

    receiving_x = 6500
    receiving_z = 7000

    #****************************** Routine ******************************

    insertionServosOpen()
    if hose_jig.gripper_open() != "success" : return "error01"
    if insertion_servos.slider_nozzle_receive() != "success" : return "error02"

    # Feed Hose
    if lubrication_feeder.close_hose_holder() != "success" : return "error41"
    if insertion_servos.holder_hose_nozzle_semi_close() != "success" : return "error41"
    lubrication_feeder.move_feeder_until_ir(speed=feeder_speed)
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error41"
    if lubrication_feeder.open_hose_holder() != "success" : return "error41"


    # Clamping and Reset Gripper
    if insertion_servos.clamp_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_joint_close() != "success" : return "error08"
    if puller_extension.open_gripper() != "success" : return "error09"


    # Homing Slider Before Movement
    if insertion_servos.slider_joint_home() != "success" : return "error10"
    if insertion_servos.slider_nozzle_receive() != "success" : return "error11"


    # Insertio Jig Home POsition
    if hose_jig.insertion_position(False) != "success" : return "error12"


    #Lubricate Hose
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error17"
    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error16"
    if lubrication_feeder.lubricate_nozzle(lubricate_nozzle_time) != "success" : return "error18"


    # Nozzle Insertion
    if insertion_jig.move_z_axis(insertion_position_z) != "success" : return "error20"
    if insertion_jig.move_x_axis(insert_nozzle) != "success" : return "error20"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_preinsertion() != "success" : return "error23.1"
    time.sleep(1)
    if insertion_servos.slider_nozzle_insertion() != "success" : return "error23"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success" : return "error24"
    if insertion_servos.clamp_nozzle_open() != "success" : return "error25"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_receive() != "success" : return "error26"


    # Go to Down Position for Hose Puller
    if insertion_jig.move_z_axis(insertion_jig_safe_zone) != "success" : return "error27"
    if insertion_jig.move_x_axis(lubricate_joint,flag=False) != "success" : return "6"


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
    if lubrication_feeder.close_hose_holder() != "success" : return "error41"
    if insertion_servos.activate_cutter() != "success" : return "error41"

    time.sleep(.5)


    # Alignment for Joint Insertion
    if hose_puller.move_y_actuator_with_speed(alignmnet_for_joint,200) != "success" : return "error13" #Aqui hare el cambio
    if insertion_servos.holder_hose_joint_semi_close() != "success" : return "error42"
    if hose_puller.move_y_axis_until_no_hose(hose_puller_y_speed_for_alignment) != "success" : return "error43"
    if insertion_servos.holder_hose_joint_close() != "success" : return "error45"


    # Lubricate Hose on Joint Area
    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "6"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error47"
    if lubrication_feeder.lubricate_joint(lubricate_joint_time) != "success" : return "error48"
    

    # Joint Insertion
    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error49"
    insertion_jig.move_z_axis(insertion_position_joint_z)
    time.sleep(0.5)
    if insertion_servos.slider_joint_preinsertion() != "success" : return "error52"
    time.sleep(1)
    if insertion_servos.slider_joint_insertion() != "success" : return "error52"
    time.sleep(.5)
    if insertion_servos.holder_hose_joint_open() != "success" : return "error53"
    if insertion_servos.clamp_joint_open() != "success" : return "error54"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success" : return "error55"


    # Finish Pulling Action
    if hose_puller.move_y_actuator(home_y) != "success" : return "error56"


    # Receiving for Finish
    if insertion_jig.move_z_axis(receiving_z,False) != "success" : return "error57"
    if insertion_jig.move_x_axis(receiving_x,False) != "success" : return "error58"
    if hose_puller.move_z_actuator(z_picking_position) != "success" : return "error59"


    # Delivering Hose
    if hose_jig.gripper_close() != "success" : return "error60"
    if puller_extension.open_gripper() != "success" : return "error61"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error62"
    if hose_jig.deliver_position() != "success" : return "error63"
    if hose_jig.gripper_open() != "success" : return "error64"
    if hose_puller.move_z_actuator(0) != "success" : return "error65"

    return "success"

#Pick Up Hose
def pickUpHose():

    global transporter_fuyus, transporter_grippers, hose_jig

    # transporter_fuyus.home_x_axis()


    if hose_jig.gripper_close() != "success" : return "001"
    if transporter_fuyus.pickHome() != "success" : return "01"

    if transporter_fuyus.moveToReceivingPosition() != "success" : return "03"

    if transporter_fuyus.pickHoseFromFirstStation() != "success" : return "04"

    if transporter_grippers.close_all_grippers() != "success" : return "05"

    time.sleep(1)

    if hose_jig.gripper_open() != "success" : return "06"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToDeliverPosition() != "success": return "03"

    if transporter_grippers.open_all_grippers() != "success": return "05"

    if transporter_fuyus.pickHome() != "success": return "07"

    if transporter_fuyus.moveToSafeSpace() != "success": return "08"

    return "success"

def testCam():
    global pick_and_place_camera
    ok = 0
    noOk = 0
    while True:
        if (pick_and_place_camera.pick_up_nozzle()) != None                                                   :
            ok += 1
        else:
            noOk += 1

        print(f"Ok : {ok} , noOk : {noOk}")
        time.sleep(2)

def main():
    """
    Ejecuta las 3 funciones principales en secuencia:
    1. pickUpHose - Recoge la manguera de la estación
    2. movePickandPlace - Recoge y entrega nozzle y joint
    3. oneCycle - Ciclo completo de inserción (nozzle, hose, joint)
    """
    if canbus is None:
        print("Error: CAN bus no está disponible. No se puede ejecutar el protocolo.")
        print("Verifica el hardware y los drivers (Linux: canbus_jetson_v2, Windows: canbus_v2).")
        sys.exit(1)

    print("=== Iniciando secuencia principal ===\n")

    # testCam()

    # 1. Move pick and place (nozzle + joint)
    print("[1/3] Ejecutando movePickandPlace()...")
    result = movePickandPlace()
    if result != "success":
        print(f"movePickandPlace falló: {result}")
        sys.exit(1)
    print("movePickandPlace: OK\n")

    # 2. One cycle (insertion routine)
    # print("[2/3] Ejecutando oneCycle()...")
    # result = oneCycle()
    # if result != "success":
    #     print(f"oneCycle falló: {result}")
    #     sys.exit(1)
    # print("oneCycle: OK\n")

    
    # 3. Pick up hose
    # print("[3/3] Ejecutando pickUpHose()...")
    # result = pickUpHose()
    # if result != "success":
    #     print(f"pickUpHose falló: {result}")
    #     sys.exit(1)
    # print("pickUpHose: OK\n")

    print("=== Secuencia principal completada correctamente ===")


if __name__ == "__main__":
    main()
