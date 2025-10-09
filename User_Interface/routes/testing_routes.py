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
from classes.lubrication_feeder import LubricationFeeder
from classes.transporter_fuyus import TransporterFuyus
from classes.transporter_grippers import TransporterGrippers

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
CANBUS_ID_LUBRICATION_FEEDER = 0x019
CANBUS_ID_TRANSPORTER_FUYUS = 0x021
CANBUS_ID_TRANSPORTER_GRIPPERS = 0x020


#Instance Declaration
hose_jig = HoseJig(canbus, CANBUS_ID_JIG)
hose_puller = HosePuller(canbus, CANBUS_ID_PULLER)
puller_extension = PullerExtension(canbus, CANBUS_ID_EXTENSION)
insertion_jig = InsertionJig(canbus, CANBUS_ID_INSERTION)
elevator_in = ElevatorIn(canbus, CANBUS_ID_ELEVATOR_IN)
pick_and_place = PickAndPlace(canbus, CANBUS_ID_PICK_AND_PLACE)
insertion_servos = InsertionServos(canbus, CANBUS_ID_INSERTION_SERVOS)
lubrication_feeder = LubricationFeeder(canbus, CANBUS_ID_LUBRICATION_FEEDER)
transporter_fuyus = TransporterFuyus(canbus, CANBUS_ID_TRANSPORTER_FUYUS)
transporter_grippers = TransporterGrippers(canbus, CANBUS_ID_TRANSPORTER_GRIPPERS)

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
    ('Transporter Grippers', transporter_grippers)

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
        return jsonify(success=False, message=f'Error in Test Action 6: {str(e)}'), 500

#Lubrication Test
@testing_bp.route('/test_action_7', methods=['POST'])
def test_action_7():
    """Test Action 7 - Lubrication Test function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if lubrication_test() != "success" : return jsonify(success=False, message=f'Error in Lubrication Test'), 500
        
        result = {
            'success': True,
            'message': 'Lubrication Test executed successfully',
            'action': 'lubrication_test',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Lubrication Test function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 7: {str(e)}'), 500

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
        if movePickandPlace(2) != "success" : return jsonify(success=False, message=f'Error in Pick and Place'), 500
        if oneCycle() != "success" : return jsonify(success=False, message=f'Error in Insertion'), 500
        if pickUpHose() != "success" : return jsonify(success=False, message=f'Error in Deliver Hose'), 500
        
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


#****************** Core Functions ***********************

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
    if elevator_in.home_gantry_z() != "success": return "error01"
    if elevator_in.home_gantry_x() != "success": return "error02"
    if elevator_in.home_gantry_y() != "success": return "error03"
    if elevator_in.home_elevator_z() != "success": return "error04"

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
    # if pick_and_place.start_left_conveyor() != "success": return "error22"
    # if pick_and_place.start_right_conveyor() != "success": return "error23"
    # time.sleep(3)
    # if pick_and_place.stop_left_conveyor() != "success": return "error24"
    # if pick_and_place.stop_right_conveyor() != "success": return "error25"

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
    global puller_extension, pick_and_place, lubrication_feeder

    # ****************************** Insertion Jig Data ******************************

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

    # ****************************** Hose Puller Data ******************************

    safe_position = 200
    home_y = 4200
    wait_y = 5930
    pickup_y = 8990
    z_home = 50

    # ****************************** Routine ******************************

    # Feed Hose

    if lubrication_feeder.close_hose_holder() != "success": return "error01"
    if lubrication_feeder.feed_hose(duration=3.15) != "success": return "error02"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_close() != "success": return "error08"
    if lubrication_feeder.open_hose_holder() != "success": return "error03"

    # Clamping and Reset Gripper
    if insertion_servos.clamp_nozzle_close() != "success": return "error08"
    if insertion_servos.clamp_joint_close() != "success": return "error19"
    if puller_extension.open_gripper() != "success": return "error20"

    # Homing Slider Before Movement
    if insertion_servos.slider_joint_home() != "success": return "error08"
    if insertion_servos.slider_nozzle_home() != "success": return "error08"

    # Insertio Jig Home POsition

    if hose_jig.insertion_position(False) != "success": return "error05"
    if insertion_jig.move_z_axis(home_position_z) != "success": return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success": return "error02"

    # Lubricate Hose

    if insertion_servos.holder_hose_nozzle_close() != "success": return "error18"
    if insertion_jig.move_z_axis(lubrication_position_z) != "success": return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success": return "error04"
    if lubrication_feeder.lubricate_nozzle(5) != "success": return "error03.1"

    # Nozzle Insertion
    if insertion_jig.move_z_axis(insertion_position_z) != "success": return "error05"
    if insertion_jig.move_x_axis(insert_nozzle) != "success": return "error06"
    if insertion_servos.holder_hose_nozzle_close() != "success": return "error07"
    if insertion_servos.clamp_nozzle_close() != "success": return "error08"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_insertion() != "success": return "error09"
    time.sleep(1)
    if insertion_servos.holder_hose_nozzle_open() != "success": return "error10"
    if insertion_servos.clamp_nozzle_open() != "success": return "error11"
    time.sleep(.5)
    if insertion_servos.slider_nozzle_home() != "success": return "error12"

    # Go to Down Position for Hose Puller
    if insertion_jig.move_z_axis(4000) != "success": return "error13"
    #
    # # Starting Prefeeder
    if lubrication_feeder.move_pre_feeder(150) != "success": return "error04"

    # Preparing Hose Puller and Hose Jig
    if hose_puller.move_z_actuator(safe_position) != "success": return "error04"
    if hose_puller.move_y_actuator(home_y) != "success": return "error03"
    if hose_jig.insertion_position() != "success": return "error05"

    # Pulling Action
    if hose_puller.move_y_actuator(pickup_y + 15) != "success": return "error06"
    if hose_puller.move_z_actuator(z_home + 5) != "success": return "error07"
    if puller_extension.close_gripper() != "success": return "error08"
    if hose_puller.move_y_actuator_with_speed(pickup_y - 500, 200) != "success": return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success": return "error10"
    if hose_puller.move_y_actuator_with_speed(wait_y + 1900, 200) != "success": return "error11"
    if hose_puller.move_z_actuator(safe_position + 42) != "success": return "error10"
    if hose_puller.move_y_actuator_with_speed(wait_y, 200) != "success": return "error11"

    # Cutting Hose
    if lubrication_feeder.close_hose_holder() != "success": return "error01"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success": return "error12"

    # Alignment for Joint Insertion
    if hose_puller.move_y_actuator_with_speed(home_y + 665, 100) != "success": return "error13"  # Aqui ahre el cambio

    # Stoping Prefeeder
    if lubrication_feeder.move_pre_feeder(0) != "success": return "error04"

    # Lubricate Hose on Joint Area
    if insertion_servos.holder_hose_joint_close() != "success": return "error18"
    if insertion_jig.move_x_axis(lubricate_joint) != "success": return "error15"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success": return "error14"
    if lubrication_feeder.lubricate_joint(5) != "success": return "error14.1"
    time.sleep(0.5)

    # Joint Insertion
    if insertion_jig.move_x_axis(insert_joint) != "success": return "error17"
    if insertion_jig.move_z_axis(insertion_position_joint_z) != "success": return "error16"
    if insertion_servos.clamp_joint_close() != "success": return "error19"
    time.sleep(0.5)
    if insertion_servos.slider_joint_insertion() != "success": return "error20"
    time.sleep(1)
    if insertion_servos.holder_hose_joint_open() != "success": return "error21"
    if insertion_servos.clamp_joint_open() != "success": return "error22"
    time.sleep(0.5)
    if insertion_servos.slider_joint_home() != "success": return "error23"

    # Finish Pulling Action

    if hose_puller.move_y_actuator(home_y) != "success": return "error13"

    # Homing for Finish
    if insertion_jig.move_z_axis(3000, False) != "success": return "error24"
    if insertion_jig.move_x_axis(0, False) != "success": return "error25"
    if hose_puller.move_z_actuator(z_home) != "success": return "error14"

    # Deliverinh Hose

    if hose_jig.gripper_close() != "success": return "error15"
    if puller_extension.open_gripper() != "success": return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success": return "error17"
    if hose_jig.deliver_position(False) != "success": return "error18"
    # if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success": return "error20"

    return "success"


#Pick Up Hose
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


    insertionServosOpen()
    insertion_servos.slider_joint_home()
    
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

    return "success"


#Test lubrication
def lubrication_test():
    global lubrication_feeder,insertion_servos

    if lubrication_feeder.lubricate_nozzle(duration=5) != "success": return "error01"
    if lubrication_feeder.lubricate_joint(duration=5) != "success": return "error02"

    return "success"


#****************** Functions for movements ***********************

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

    # #****************************** Routine ******************************
    #
    # # Homing Devices
    #
    # if insertion_servos.slider_joint_home() != "success": return "error04"
    # if insertion_servos.slider_nozzle_home() != "success": return "error05"
    #
    # if insertion_jig.home_x_axis_go_home() != "success": return "error06"
    # if insertion_jig.home_z_axis_go_home() != "success": return "error07"
    #
    # if insertion_servos.clamp_nozzle_open() != "success": return "error09"
    # if insertion_servos.clamp_joint_open() != "success": return "error10"
    # if insertion_servos.cutter_open() != "success": return "error11"
    # if insertion_servos.holder_hose_joint_open() != "success": return "error12"
    # if insertion_servos.holder_hose_nozzle_open() != "success": return "error13"
    #
    # # Feed Hose
    #
    # if lubrication_feeder.close_hose_holder() != "success": return "error01"
    # if lubrication_feeder.feed_hose(duration=3.15) != "success": return "error02"
    # time.sleep(1)
    # if insertion_servos.holder_hose_nozzle_close() != "success": return "error08"
    # if lubrication_feeder.open_hose_holder() != "success": return "error03"
    #
    #
    # # Align Nozzle Position
    #
    #
    # if insertion_jig.move_z_axis(home_position_z) != "success": return "error14"
    # if insertion_jig.move_x_axis(home_position_x) != "success": return "error15"
    #
    # # Lubricate Hose Nozzle Area
    #
    # if insertion_servos.holder_hose_nozzle_close() != "success": return "error16"
    #
    # if insertion_jig.move_x_axis(lubricate_nozzle) != "success": return "error17"
    # if insertion_jig.move_z_axis(lubrication_position_z) != "success": return "error18"
    #
    # if lubrication_feeder.lubricate_nozzle(5) != "success": return "error19"
    #
    # # Insert Nozzle
    #
    # if insertion_jig.move_z_axis(insertion_position_z) != "success": return "error20"
    # if insertion_jig.move_x_axis(insert_nozzle) != "success": return "error21"
    #
    # if insertion_servos.holder_hose_nozzle_close() != "success": return "error22"
    # if insertion_servos.clamp_nozzle_close() != "success": return "error23"
    # time.sleep(.5)
    # if insertion_servos.slider_nozzle_insertion() != "success": return "error24"
    # time.sleep(1)
    # if insertion_servos.holder_hose_nozzle_open() != "success": return "error25"
    # if insertion_servos.clamp_nozzle_open() != "success": return "error26"
    # time.sleep(.5)
    # if insertion_servos.slider_nozzle_home() != "success": return "error27"
    #
    #
    # # Safe Position
    #
    #
    # if insertion_servos.clamp_joint_close() != "success": return "error28"
    # if insertion_jig.move_z_axis(home_position_z) != "success": return "error29"
    #
    # # Lubricate Hose Joint Area
    #
    # if insertion_jig.move_x_axis(lubricate_joint) != "success": return "error30"
    # if insertion_jig.move_z_axis(librication_position_joint_z) != "success": return "error31"
    # if lubrication_feeder.lubricate_joint(5) != "success": return "error32"
    #
    # #Insert Joint
    #
    # if insertion_jig.move_z_axis(insertion_position_joint_z) != "success": return "error33"
    # if insertion_jig.move_x_axis(insert_joint) != "success": return "error34"
    # if insertion_servos.holder_hose_joint_close() != "success": return "error35"
    # if insertion_servos.clamp_joint_close() != "success": return "error36"
    # time.sleep(0.5)
    # if insertion_servos.slider_joint_insertion() != "success": return "error37"
    # time.sleep(1)
    # if insertion_servos.holder_hose_joint_open() != "success": return "error38"
    # if insertion_servos.clamp_joint_open() != "success": return "error39"
    # time.sleep(0.5)
    # if insertion_servos.slider_joint_home() != "success": return "error40"
    #
    # # Back to Home
    #
    # if insertion_jig.move_z_axis(0) != "success": return "error41"
    # if insertion_jig.move_x_axis(0) != "success": return "error42"

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


#Test transporter and grippers
def moveTransporter():
    global transporter_fuyus, transporter_grippers,hose_jig

    # print(hose_jig.read_deliver())
    
    # Test TransporterFuyus functionality
    print("\n--- Testing TransporterFuyus ---")
    
    # Home X axis
    print("Homing X axis...")
    result = transporter_fuyus.home_x_axis()
    print(f"Result: {result}")
    
    # Move X axis to different positions
    print("Moving X axis to Taping Hose Jig ")
    result = transporter_fuyus.moveToTapingHoseJig()
    print(f"Result: {result}")

    print("Moving X axis to Stamper Hose Jig ")
    result = transporter_fuyus.moveToStamperHoseJig()
    print(f"Result: {result}")

    print("Moving X axis to Deliver ")
    result = transporter_fuyus.moveToDeliverPosition()
    print(f"Result: {result}")

    print("Moving X axis to Receiving ")
    result = transporter_fuyus.moveToReceivingPosition()
    print(f"Result: {result}")

    
    # Move all steppers
    print("Moving all steppers to position 1000...")
    result = transporter_fuyus.move_all_steppers(150000,1)
    print(f"Result: {result}")

    result = transporter_fuyus.pickHoseFromFirstStation()
    print(f"Result: {result}")

    result = transporter_fuyus.pickHome()
    # Test TransporterGrippers functionality
    print("\n--- Testing TransporterGrippers ---")
    
    # Open and close all grippers

    print("Closing all grippers...")
    result = transporter_grippers.close_all_grippers()
    print(f"Result: {result}")

    time.sleep(2)

    print("Opening all grippers...")
    result = transporter_grippers.open_all_grippers()
    print(f"Result: {result}")
    

def testIR():
    global elevator_in

    print(elevator_in.check_ir_sensor_status())

    print("Testing home_elevator_z...")
    result = elevator_in.home_elevator_z()
    print(f"Result: {result}")
