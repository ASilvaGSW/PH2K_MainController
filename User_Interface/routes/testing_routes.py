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
from classes.pick_and_place import PickAndPlace
from classes.insertion_servos import InsertionServos

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
CANBUS_ID_PICK_AND_PLACE = 0x191
CANBUS_ID_INSERTION_SERVOS = 0x002

#Instance Declaration
hose_jig = HoseJig(canbus, CANBUS_ID_JIG)
hose_puller = HosePuller(canbus, CANBUS_ID_PULLER)
puller_extension = PullerExtension(canbus, CANBUS_ID_EXTENSION)
insertion_jig = InsertionJig(canbus, CANBUS_ID_INSERTION)
elevator_in = ElevatorIn(canbus, CANBUS_ID_ELEVATOR_IN)
pick_and_place = PickAndPlace(canbus, CANBUS_ID_PICK_AND_PLACE)
insertion_servos = InsertionServos(canbus, CANBUS_ID_INSERTION_SERVOS)

# Device list for batch operations
devices = [
    ('Hose Jig', hose_jig),
    ('Hose Puller', hose_puller),
    ('Puller Extension', puller_extension),
    ('Insertion Jig', insertion_jig),
    ('Elevator In', elevator_in),
    ('Pick and Place', pick_and_place),
    ('Insertion Servos', insertion_servos)
]

# Helper functions for batch device operations
def execute_device_operation(operation_name, operation_func):
    """Execute an operation on all devices and return failed devices list"""
    failed_devices = []
    
    for device_name, device in devices:
        try:
            status = operation_func(device)
            if status != "success":
                failed_devices.append(f'{device_name} (Status: {status})')
        except Exception as e:
            failed_devices.append(f'{device_name} (Error: {str(e)})')
    
    return failed_devices

def test_all_devices_heartbeat():
    """Test heartbeat for all devices and return results"""
    return execute_device_operation("heartbeat", lambda device: device.send_heartbeat())

# Create blueprint for testing routes
testing_bp = Blueprint('testing', __name__, url_prefix='/api')

#Check Canbus
@testing_bp.route('/test_action_1', methods=['POST'])
def test_action_1():
    """Test Action 1 - Play/Start function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        if canbus is not None:
            # Check if canbus is properly started
            print(f"Canbus status - is_started: {canbus.is_started}")
            print(f"Canbus device: {canbus.device}")
            print(f"Canbus channel: {canbus.channel}")
            
            # Try to restart canbus if not started
            if not canbus.is_started:
                print("Canbus not started, attempting to restart...")
                restart_result = canbus.start_canbus()
                print(f"Restart result: {restart_result}")
                if not restart_result:
                    return jsonify(success=False, message='Failed to restart CAN bus'), 500
            
            print("Sending heartbeat...")
            status = hose_jig.send_heartbeat()
            print(f"Heartbeat status: {status}")

            if status == "success":
                return jsonify(success=True, message='Hose Jig heartbeat sent successfully'), 200
            else:
                return jsonify(success=False, message=f'Failed to send Hose Jig heartbeat - Status: {status}'), 500

        else:
            return jsonify(success=False, message='Canbus not enabled'), 500
        
    except Exception as e:
        print(f"Exception in test_action_1: {str(e)}")
        return jsonify(success=False, message=f'Error in Test Action 1: {str(e)}'), 500

#Check all MCU
@testing_bp.route('/test_action_2', methods=['POST'])
def test_action_2():
    """Test Action 2 - Send heartbeat to all devices"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        failed_devices = test_all_devices_heartbeat()
        
        if failed_devices:
            return jsonify(
                success=False, 
                message=f'Failed to send heartbeat to: {", ".join(failed_devices)}'
            ), 500
        
        return jsonify(success=True, message='All heartbeats sent successfully'), 200
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 2: {str(e)}'), 500

#Feed Material
@testing_bp.route('/test_action_3', methods=['POST'])
def test_action_3():
    """Test Action 3 - Feed Material function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if moveElevatorIn() != "success" : return jsonify(success=False, message=f'Error in Test Action 3: {str(e)}'), 500
        
        result = {
            'success': True,
            'message': 'Feed Material executed successfully',
            'action': 'feed_material',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Feed Material function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 3'), 500

#Pick and Place
@testing_bp.route('/test_action_4', methods=['POST'])
def test_action_4():
    """Test Action 4 - Pick and Place function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if movePickandPlace() != "success" : return jsonify(success=False, message=f'Error in Pick and Place'), 500
        
        result = {
            'success': True,
            'message': 'Pick and Place executed successfully',
            'action': 'pick_and_place',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Pick and Place function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 4'), 500

#Insertion
@testing_bp.route('/test_action_5', methods=['POST'])
def test_action_5():
    """Test Action 5 - Insertion action function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if insertionRoutine() != "success" : return jsonify(success=False, message=f'Error in Insertion: {str(e)}'), 500
        
        result = {
            'success': True,
            'message': 'Insertion executed successfully',
            'action': 'insertion',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Insertion function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 5: {str(e)}'), 500

#Hose Pulling
@testing_bp.route('/test_action_6', methods=['POST'])
def test_action_6():
    """Test Action 6 - Hose Pulling function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if moveHosepuller() != "success" : return jsonify(success=False, message=f'Error in Hose Pulling: {str(e)}'), 500
        
        result = {
            'success': True,
            'message': 'Hose Pulling executed successfully',
            'action': 'hose_pulling',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Hose Pulling function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 6: {str(e)}'), 500

#Full Routine
@testing_bp.route('/test_action_7', methods=['POST'])
def test_action_7():
    """Test Action 7 - Main Routine function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if moveElevatorIn() != "success" : return jsonify(success=False, message=f'Error in Elevator: {str(e)}'), 500
        if movePickandPlace() != "success" : return jsonify(success=False, message=f'Error in Pick and Place: {str(e)}'), 500
        if insertionRoutine() != "success" : return jsonify(success=False, message=f'Error in Insertion: {str(e)}'), 500
        if moveHosepuller() != "success" : return jsonify(success=False, message=f'Error in Hose Pulling: {str(e)}'), 500
        
        result = {
            'success': True,
            'message': 'Full Routine executed successfully',
            'action': 'full_routine',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Full Routine function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 7: {str(e)}'), 500

@testing_bp.route('/test_action_8', methods=['POST'])
def test_action_8():
    """Test Action 8 - One Cycle function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if oneCycle() != "success" : return jsonify(success=False, message=f'Error in One Cycle'), 500
        
        result = {
            'success': True,
            'message': 'One Cycle executed successfully',
            'action': 'one_cycle',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'One Cycle function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 8: {str(e)}'), 500

@testing_bp.route('/canbus_diagnostic', methods=['GET'])
def canbus_diagnostic():
    """Diagnostic route to check CAN bus status"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        diagnostic_info = {
            'canbus_exists': canbus is not None,
            'canbus_started': canbus.is_started if canbus else False,
            'device_info': str(canbus.device) if canbus else None,
            'channel_info': str(canbus.channel) if canbus else None,
            'hose_jig_exists': hose_jig is not None,
            'canbus_id': hose_jig.canbus_id if hose_jig else None
        }
        
        # Try to list available devices
        if canbus:
            try:
                devices = canbus.device_list()
                diagnostic_info['available_devices'] = str(devices) if devices else 'No devices found'
            except Exception as e:
                diagnostic_info['device_list_error'] = str(e)
        
        return jsonify({
            'success': True,
            'message': 'CAN bus diagnostic completed',
            'diagnostic': diagnostic_info
        })
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in diagnostic: {str(e)}'), 500

@testing_bp.route('/canbus_reinit', methods=['POST'])
def canbus_reinit():

    """Reinitialize CAN bus connection"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    global canbus, hose_jig
    
    try:
        # Close existing connection if it exists
        if canbus and canbus.is_started:
            try:
                canbus.close_canbus()
                print("Closed existing CAN bus connection")
            except Exception as e:
                print(f"Error closing existing connection: {e}")
        
        # Create new CAN bus instance
        canbus = Canbus()
        start_result = canbus.start_canbus()
        
        if start_result:
            # Recreate hose_jig with new canbus instance
            hose_jig = HoseJig(canbus, 0x0CA)
            
            return jsonify({
                'success': True,
                'message': 'CAN bus reinitialized successfully',
                'canbus_started': canbus.is_started
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Failed to reinitialize CAN bus',
                'canbus_started': False
            })
        
    except Exception as e:
        print(f"Error reinitializing CAN bus: {e}")
        return jsonify(success=False, message=f'Error reinitializing CAN bus: {str(e)}'), 500

#Feeding Material
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
    
    if elevator_in.move_gantry_array(pick_left) != "success" : return "error01"      

    if elevator_in.close_gripper() != "success" : return "error02"

    if elevator_in.move_gantry_z(transportation_high) != "success" : return "error03"

    if elevator_in.move_gantry_array(deliver_left) != "success" : return "error04"

    if elevator_in.move_gantry_z(cassette_deliver_high) != "success" : return "error05"

    if elevator_in.open_gripper() != "success" : return "error06"

    if elevator_in.move_gantry_z(safety_high) != "success" : return "error07"

    if elevator_in.move_gantry_array(home) != "success" : return "error08"

    # #Right Side

    if elevator_in.move_gantry_array(pick_right) != "success" : return "error09"

    if elevator_in.close_gripper() != "success" : return "error10"

    if elevator_in.move_gantry_z(transportation_high) != "success" : return "error11"

    if elevator_in.move_gantry_array(deliver_right) != "success" : return "error12"

    if elevator_in.move_gantry_z(cassette_deliver_high) != "success" : return "error13"

    if elevator_in.open_gripper() != "success" : return "error14"

    if elevator_in.move_gantry_z(safety_high) != "success" : return "error15"

    # #conveyors

    if pick_and_place.start_left_conveyor() != "success" : return "error17"
    if pick_and_place.start_right_conveyor() != "success" : return "error18"

    if elevator_in.move_gantry_array(home) != "success" : return "error16"

    time.sleep(2)

    if pick_and_place.stop_left_conveyor() != "success" : return "error19"
    if pick_and_place.stop_right_conveyor() != "success" : return "error20"

    return "success"


#Doing Pick and Place
def movePickandPlace():

    global pick_and_place, insertion_servos, insertion_jig

    # Homing Insertion Jig
    if insertion_servos.slider_joint_home() != "success" : return "error01"
    if insertion_jig.move_z_axis(5000) != "success" : return "error02"
    if insertion_jig.move_x_axis(-8500) != "success" : return "error03"


    #Home Pick and Place
    if pick_and_place.move_actuator_z(0) != "success" : return "error04"   
    if pick_and_place.move_actuator_x(0) != "success" : return "error05"
    if pick_and_place.open_gripper() != "success" : return "error06"

    # #Pick Nozzle
    if pick_and_place.move_actuator_x(-1800) != "success" : return "error07"
    if pick_and_place.move_actuator_z(-1250) != "success" : return "error08"
    if pick_and_place.close_gripper() != "success" : return "error09"

    # #Safe Position 1
    if pick_and_place.move_actuator_z(-1000) != "success" : return "error10"

    # #Place Nozzle
    if pick_and_place.move_actuator_x(-4090) != "success" : return "error11"
    if pick_and_place.move_actuator_z(-1380) != "success" : return "error12"
    if pick_and_place.open_gripper() != "success" : return "error13"

    # #Safe Position 2
    if pick_and_place.move_actuator_z(-1000) != "success" : return "error14"    

    # #Pick Joint
    if pick_and_place.move_actuator_x(-850) != "success" : return "error15"
    if pick_and_place.move_actuator_z(-1250) != "success" : return "error16"
    if pick_and_place.close_gripper() != "success" : return "error17"

    # #Safe Position 3
    if pick_and_place.move_actuator_z(-1000) != "success" : return "error18"

    # #Place Joint
    if pick_and_place.move_actuator_x(-4390) != "success" : return "error19"
    if pick_and_place.move_actuator_z(-1320) != "success" : return "error20"
    if pick_and_place.open_gripper() != "success" : return "error21"

    # #Safe Position 4
    if pick_and_place.move_actuator_z(0) != "success" : return "error22"
    if pick_and_place.move_actuator_x(0) != "success" : return "error23"
    if pick_and_place.open_gripper() != "success" : return "error24"

    return "success"


# Making Insertions
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

    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error02"

    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error04"

    if insertion_jig.move_z_axis(insertion_position_z) != "success" : return "error05"
    if insertion_jig.move_x_axis(insert_nozzle) != "success" : return "error06"
  
    
    #Nozzle Insertion


    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success" : return "error09"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success" : return "error10"
    if insertion_servos.clamp_nozzle_open() != "success" : return "error11"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_home() != "success" : return "error12"

    if insertion_servos.activate_cutter() != "success" : return "error13"

    #Moving to Joint

    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error14"
    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error15"

    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success" : return "error16"
    if insertion_jig.move_x_axis(insert_joint) != "success" : return "error17"
   

    #Clamp Insertion

    if insertion_servos.holder_hose_joint_close() != "success" : return "error18"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success" : return "error20"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success" : return "error21"
    if insertion_servos.clamp_joint_open() != "success" : return "error22"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success" : return "error23"

    if insertion_jig.move_z_axis(0) != "success" : return "error24"
    if insertion_jig.move_x_axis(0) != "success" : return "error25"

    return "success"


# Pulling Hose
def moveHosepuller():
    global hose_puller, hose_jig, puller_extension,insertion_servos, insertion_jig

    safe_position = 200
    home_y = 4200
    wait_y = 6000
    pickup_y = 9060
    z_home = 50

    if insertion_jig.move_z_axis(0) != "success" : return "error01"
    if insertion_jig.move_x_axis(0) != "success" : return "error02"

    #****************************** Hose puller ******************************
    # hose_puller.move_y_actuator(0)
    # hose_puller.move_z_actuator(0)

    if hose_puller.move_y_actuator(home_y) != "success" : return "error03"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error04"
    if hose_jig.insertion_position() != "success" : return "error05"
    if hose_puller.move_y_actuator(pickup_y) != "success" : return "error06"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error07"
    if puller_extension.close_gripper() != "success" : return "error08"
    if hose_puller.move_y_actuator(pickup_y-500) != "success" : return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error10"
    if hose_puller.move_y_actuator(wait_y) != "success" : return "error11"
    if insertion_servos.activate_cutter() != "success" : return "error12"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error13"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error14"
    if hose_jig.gripper_close() != "success" : return "error15"
    if puller_extension.open_gripper() != "success" : return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error17"
    if hose_jig.deliver_position() != "success" : return "error18"
    if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success" : return "error20"

    return "success"

    
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
