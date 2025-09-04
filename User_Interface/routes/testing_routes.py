from flask import Blueprint, request, jsonify, session
from datetime import datetime
import sys
import os
import time

# Add the main_controller directory to Python path
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'main_controller'))

# Import functions from main.py
from main import (
    moveHosepuller,
    insertionRoutine,
    moveElevatorIn,
    movePickandPlace,
    oneCycle,
    lubrication_test,
    testHome,
    moveTransporter
)

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

#One Cycle
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

#Homing
@testing_bp.route('/test_action_9', methods=['POST'])
def test_action_9():
    """Test Action 9 - Home function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if testHome() != "success" : return jsonify(success=False, message=f'Error in Home'), 500
        
        result = {
            'success': True,
            'message': 'Home executed successfully',
            'action': 'home',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Home function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 9: {str(e)}'), 500

#Lubrication
@testing_bp.route('/test_action_10', methods=['POST'])
def test_action_10():
    """Test Action 10 - Lubrication function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        if lubrication_test() != "success" : return jsonify(success=False, message=f'Error in Lubrication'), 500
        
        result = {
            'success': True,
            'message': 'Lubrication executed successfully',
            'action': 'lubrication',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Lubrication function executed'
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


#****************** Functions imported from main.py ***********************
# All movement functions are now imported from main.py to avoid duplication


# insertionRoutine function is now imported from main.py


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


# moveElevatorIn function is now imported from main.py


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


# movePickandPlace function is now imported from main.py


# oneCycle function is now imported from main.py

# moveTransporter function is now imported from main.py
    
  
# testHome and lubrication_test functions are now imported from main.py