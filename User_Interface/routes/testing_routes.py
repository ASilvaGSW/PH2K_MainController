from flask import Blueprint, request, jsonify, session
from datetime import datetime
import sys
import os
import time

import threading

# Global variables for operation control
pause_event = threading.Event()
operation_status = {
    "state": "idle", # idle, running, paused, completed, error
    "message": ""
}
first_pick_after_align = False

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

# Device list for batch operations
devices = [
    ('Hose Jig', hose_jig),
    ('Hose Puller', hose_puller),
    ('Puller Extension', puller_extension),
    ('Insertion Jig', insertion_jig),
    ('Elevator In', elevator_in),
    ('Pick and Place', pick_and_place),
    ('Insertion Servos', insertion_servos),
    ('Lubrication Feeder', lubrication_feeder),
    ('Transporter Fuyus', transporter_fuyus),
    ('Transporter Grippers', transporter_grippers),
    ('Pick and Place Camera', pick_and_place_camera)

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

@testing_bp.route('/status', methods=['GET'])
def get_status():
    return jsonify(operation_status)

@testing_bp.route('/resume', methods=['POST'])
def resume_operation():
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    pause_event.set()
    return jsonify(success=True)

#Receive Material
@testing_bp.route('/test_action_1', methods=['POST'])
def test_action_1():
    """Test Action 1 - Receive Material function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO2: Add your CAN bus and Python actions here
        if ReceiveMaterial() != "success" : return jsonify(success=False, message=f'Error in Receive Material'), 500
        
        result = {
            'success': True,
            'message': 'Receive Material executed successfully',
            'action': 'receive_material',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Receive Material function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 1: {str(e)}'), 500

#Align Component
@testing_bp.route('/test_action_2', methods=['POST'])
def test_action_2():
    """Test Action 2 - Align Component function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO2: Add your CAN bus and Python actions here
        if alignComponent() != "success" : return jsonify(success=False, message=f'Error in Align Component'), 500
        
        result = {
            'success': True,
            'message': 'Align Component executed successfully',
            'action': 'align_component',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Align Component function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 2: {str(e)}'), 500

#Test Home
@testing_bp.route('/test_action_3', methods=['POST'])
def test_action_3():
    """Test Action 3 - Test Home function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if testHome() != "success" : return jsonify(success=False, message=f'Error in Test Home'), 500
        
        result = {
            'success': True,
            'message': 'Test Home executed successfully',
            'action': 'test_home',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Test Home function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 3'), 500

#Feed Cassette
@testing_bp.route('/test_action_4', methods=['POST'])
def test_action_4():
    """Test Action 4 - Feed Cassette function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if moveElevatorIn() != "success" : return jsonify(success=False, message=f'Error in Feed Cassette'), 500
        
        result = {
            'success': True,
            'message': 'Feed Cassette executed successfully',
            'action': 'feed_cassette',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Feed Cassette function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 4'), 500

#Align Cassette
@testing_bp.route('/test_action_5', methods=['POST'])
def test_action_5():
    """Test Action 5 - Align Cassette function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if alignCassette() != "success" : return jsonify(success=False, message=f'Error in Align Cassette'), 500
        
        result = {
            'success': True,
            'message': 'Align Cassette executed successfully',
            'action': 'align_cassette',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Align Cassette function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 5: {str(e)}'), 500

#Pick and Place
@testing_bp.route('/test_action_6', methods=['POST'])
def test_action_6():
    """Test Action 6 - Pick and Place function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        global operation_status
        operation_status["state"] = "running"
        if movePickandPlace(need=True) != "success" : 
            operation_status["state"] = "error"
            return jsonify(success=False, message=f'Error in Pick and Place'), 500
        
        operation_status["state"] = "completed"
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
        return jsonify(success=False, message=f'Error in Test Action 6: {str(e)}'), 500

# Test Action 7 Removed

#Insertion Full Cycle
@testing_bp.route('/test_action_8', methods=['POST'])
def test_action_8():
    """Test Action 8 - Insertion Full Cycle function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if oneCycle() != "success" : return jsonify(success=False, message=f'Error in Insertion Full Cycle'), 500
        
        result = {
            'success': True,
            'message': 'Insertion Full Cycle executed successfully',
            'action': 'insertion_full_cycle',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Insertion Full Cycle function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 8: {str(e)}'), 500

#Deliver Hose
@testing_bp.route('/test_action_9', methods=['POST'])
def test_action_9():
    """Test Action 9 - Deliver Hose function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if pickUpHose() != "success" : return jsonify(success=False, message=f'Error in Deliver Hose'), 500
        
        result = {
            'success': True,
            'message': 'Deliver Hose executed successfully',
            'action': 'deliver_hose',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Deliver Hose function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 9: {str(e)}'), 500

#Full Routine
@testing_bp.route('/test_action_10', methods=['POST'])
def test_action_10():
    """Test Action 10 - Full Routine function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        global operation_status
        operation_status["state"] = "running"
        if movePickandPlace(need=True) != "success" : 
            operation_status["state"] = "error"
            return jsonify(success=False, message=f'Error in Pick and Place'), 500
        if oneCycle() != "success" : 
            operation_status["state"] = "error"
            return jsonify(success=False, message=f'Error in Insertion'), 500
        if pickUpHose() != "success" : 
            operation_status["state"] = "error"
            return jsonify(success=False, message=f'Error in Deliver Hose'), 500
        
        operation_status["state"] = "completed"
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
        return jsonify(success=False, message=f'Error in Test Action 10: {str(e)}'), 500

#Check Canbus
@testing_bp.route('/check_canbus', methods=['POST'])
def check_canbus():
    """Check Canbus function - Heartbeat all devices"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        if canbus is not None:
            # Check if canbus is properly started
            if not canbus.is_started:
                print("Canbus not started, attempting to restart...")
                restart_result = canbus.start_canbus()
                if not restart_result:
                    return jsonify(success=False, message='Failed to restart CAN bus'), 500
            
            results = []
            all_success = True
            
            # Iterate through all devices and send heartbeat
            for device_name, device in devices:
                try:
                    print(f"Sending heartbeat to {device_name}...")
                    status = device.send_heartbeat()
                    print(f"{device_name} heartbeat status: {status}")
                    
                    results.append({
                        'device': device_name,
                        'status': 'success' if status == 'success' else 'failed',
                        'message': status
                    })
                    
                    if status != 'success':
                        all_success = False
                        
                except Exception as e:
                    print(f"Error checking {device_name}: {str(e)}")
                    results.append({
                        'device': device_name,
                        'status': 'error',
                        'message': str(e)
                    })
                    all_success = False
            
            message = 'All devices responded successfully' if all_success else 'Some devices failed to respond'
            
            return jsonify({
                'success': all_success,
                'message': message,
                'results': results
            })

        else:
            return jsonify(success=False, message='Canbus not enabled'), 500
        
    except Exception as e:
        print(f"Exception in check_canbus: {str(e)}")
        return jsonify(success=False, message=f'Error in Check Canbus: {str(e)}'), 500

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

@testing_bp.route('/test_action_11', methods=['POST'])
def test_action_11():
    """Test Action 11 - Elevator Out Sequence"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        if elevator_out.move_elevator_to_reception() != "success":
            return jsonify(success=False, message='Error moving elevator to reception'), 500
        
        
        if elevator_out.open_servo() != "success":
            return jsonify(success=False, message='Error opening servo'), 500
            

        # Move elevator down relatively (e.g., 1000 units)
        if elevator_out.move_elevator_relative(1000) != "success":
            return jsonify(success=False, message='Error moving elevator down'), 500
            
        time.sleep(1) # Wait for the elevator to move down

        if elevator_out.release_servo() != "success":
            return jsonify(success=False, message='Error releasing servo'), 500

        result = {
            'success': True,
            'message': 'Elevator Out sequence executed successfully',
            'action': 'elevator_out_sequence',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Elevator Out sequence function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 11: {str(e)}'), 500

#****************** Core Functions ***********************

#Receive Material
def ReceiveMaterial():
    global elevator_in

    home = [0, 0, 0]
    safety_high = -300
    transportation_high = -500

    # wait
    deliver_left = [-225, -1800, -500]

    # Homing
    if elevator_in.home_elevator_z() != "success": return "error04"

    if elevator_in.move_gantry_z(transportation_high) != "success": return "error08"
    if elevator_in.move_gantry_array(deliver_left) != "success": return "error09"

    return "success"



#Move Elevator In
def moveElevatorIn():

    global elevator_in,pick_and_place

    home = [0,0,0]
    safety_high = -300
    cassette_deliver_high = -600
    transportation_high = -490

    #left
    pick_left = [-225,-319,-815]
    pick_left = [-225,-313,-815]
    deliver_left = [-250,-1800,-500]
    
    #Right
    pick_right = [-1358,-319,-835]
    pick_right = [-1355,-319,-835]
    deliver_right = [-1365,-1800,-500]
    deliver_right2 = [-1410,-1800,-500]

    # #Homing
    # if elevator_in.home_gantry_z() != "success": return "error01"
    # if elevator_in.home_gantry_x() != "success": return "error02"
    # if elevator_in.home_gantry_y() != "success": return "error03"
    if elevator_in.move_gantry_array([0,0,0]) != "success": return "error09"

    #Aligning Cassette
    if elevator_in.move_elevator_until_sensor(0, 200) != "success": return "error05"

    #Left Side
    if elevator_in.move_gantry_array(pick_left) != "success": return "error06"
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

    return "success"


#Align Component
def alignComponent():
    global first_pick_after_align


    pick_and_place_camera.alignment_joint()
    pick_and_place_camera.alignment_nozzle()
    # pick_and_place_camera.alignment_joint()
    # pick_and_place_camera.alignment_nozzle()

    first_pick_after_align = True

    return "success"


#Move Pick and Place
def movePickandPlace(need=True):
    global operation_status, pause_event
    global pick_and_place, insertion_servos, insertion_jig, pick_and_place_camera,first_pick_after_align

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
    
    # Get index from camera for Nozzle
    n_nozzle = pick_and_place_camera.pick_up_nozzle()
    
    # Check for empty row (0xFF)
    if n_nozzle == 255:
        print("Nozzle row empty, aligning...")
        pick_and_place_camera.alignment_nozzle()
        first_pick_after_align = True
        # pick_and_place_camera.alignment_nozzle()
        # Retry after alignment
        n_nozzle = pick_and_place_camera.pick_up_nozzle()
        if n_nozzle == 255:
            return "error09_nozzle_empty"
            
    if n_nozzle is None:
        return "error09_camera_fail"
        
    # Ensure index is within bounds
    if n_nozzle >= len(nozzle_x):
        n_nozzle = len(nozzle_x) - 1 # Fallback or error? defaulting to last valid or erroring out
        # return f"error09_index_{n_nozzle}" # Let's try to proceed or return error. Returning error is safer.
        return f"error09_index_{n_nozzle}"

    if pick_and_place.move_actuator_x(nozzle_x[n_nozzle]- gap) != "success": return "error09"


    if first_pick_after_align:
        if pick_and_place.move_actuator_z(nozzle_high + 100) != "success": return "error10"
        
        # Pause for user confirmation via UI
        print("Pausing for user confirmation (Nozzle)...")
        operation_status["state"] = "paused"
        operation_status["message"] = "Waiting for user confirmation to pick nozzle. Please adjust if necessary and click Continue."
        pause_event.clear()
        pause_event.wait() # Blocks here until /resume is called
        operation_status["state"] = "running"
        operation_status["message"] = "Resuming operation..."

    if pick_and_place.move_actuator_z(nozzle_high+zgap) != "success": return "error10"
    

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
    
    # Get index from camera for Joint
    n_joint = pick_and_place_camera.pick_up_joint()
    
    # Check for empty row (0xFF)
    if n_joint == 255:
        print("Joint row empty, aligning...")
        pick_and_place_camera.alignment_joint()
        first_pick_after_align = True
        # pick_and_place_camera.alignment_joint()
        # Retry after alignment
        n_joint = pick_and_place_camera.pick_up_joint()
        if n_joint == 255:
            return "error21_joint_empty"
            
    if n_joint is None:
        return "error21_camera_fail"

    # Ensure index is within bounds
    if n_joint >= len(joint_x):
        return f"error21_index_{n_joint}"

    if pick_and_place.move_actuator_x(joint_x[n_joint]-gap) != "success": return "error21"

    if first_pick_after_align:
        if pick_and_place.move_actuator_z(joint_high + 100) != "success": return "error22"
        
        # Pause for user confirmation via UI
        print("Pausing for user confirmation (Joint)...")
        operation_status["state"] = "paused"
        operation_status["message"] = "Waiting for user confirmation to pick joint. Please adjust if necessary and click Continue."
        pause_event.clear()
        pause_event.wait() # Blocks here until /resume is called
        operation_status["state"] = "running"
        operation_status["message"] = "Resuming operation..."

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

    first_pick_after_align = False

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

    insertion_position_z = 378 + offset_z
    insert_nozzle = 2890 + offset_x

    librication_position_joint_z = 380 + offset_z
    lubricate_joint = 7420 + offset_x

    insertion_position_joint_z = 378 + offset_z
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
    feed_hose_time = 3
    lubricate_nozzle_time = 0.05
    lubricate_joint_time = 0.05
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
    time.sleep(.5)
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
    global lubrication_feeder,insertion_servos

    if lubrication_feeder.lubricate_nozzle(duration=1.5) != "success": return "error01"
    if lubrication_feeder.lubricate_joint(duration=1.5) != "success": return "error02"

    return "success"


#****************** Functions for movements ***********************


#Insertion Routine
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

