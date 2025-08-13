from flask import Blueprint, request, jsonify, session
from datetime import datetime
import sys
import os

# Add the main_controller directory to Python path
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'main_controller'))

from classes.hose_jig import HoseJig

# Now we can import the Canbus class
try:
    from classes.canbus import Canbus
    print("Canbus class found. CAN bus functionality will be enabled.")

    canbus = Canbus()
    start_result = canbus.start_canbus()
    print(f"Initial CAN bus start result: {start_result}")
    print(f"CAN bus is_started: {canbus.is_started}")

except ImportError as e:
    # Fallback if import fails - we'll handle this gracefully
    print(f"Canbus class not found: {e}. CAN bus functionality will be disabled.")
    Canbus = None
    canbus = None
except Exception as e:
    print(f"Error initializing CAN bus: {e}")
    canbus = None

hose_jig = HoseJig(canbus,0x0CA)

# Create blueprint for testing routes
testing_bp = Blueprint('testing', __name__, url_prefix='/api')

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

@testing_bp.route('/test_action_2', methods=['POST'])
def test_action_2():
    """Test Action 2 - Stop function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 2 executed successfully',
            'action': 'stop',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Stop function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 2: {str(e)}'), 500

@testing_bp.route('/test_action_3', methods=['POST'])
def test_action_3():
    """Test Action 3 - Refresh/Reload function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 3 executed successfully',
            'action': 'refresh_reload',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Refresh/Reload function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 3: {str(e)}'), 500

@testing_bp.route('/test_action_4', methods=['POST'])
def test_action_4():
    """Test Action 4 - Settings/Configuration function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 4 executed successfully',
            'action': 'settings_config',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Settings/Configuration function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 4: {str(e)}'), 500

@testing_bp.route('/test_action_5', methods=['POST'])
def test_action_5():
    """Test Action 5 - Lightning/Quick action function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 5 executed successfully',
            'action': 'quick_action',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Lightning/Quick action function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 5: {str(e)}'), 500

@testing_bp.route('/test_action_6', methods=['POST'])
def test_action_6():
    """Test Action 6 - Shield/Security check function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 6 executed successfully',
            'action': 'security_check',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Shield/Security check function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 6: {str(e)}'), 500

@testing_bp.route('/test_action_7', methods=['POST'])
def test_action_7():
    """Test Action 7 - CPU/Processing function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 7 executed successfully',
            'action': 'cpu_processing',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'CPU/Processing function executed'
            }
        }
        return jsonify(result)
        
    except Exception as e:
        return jsonify(success=False, message=f'Error in Test Action 7: {str(e)}'), 500

@testing_bp.route('/test_action_8', methods=['POST'])
def test_action_8():
    """Test Action 8 - Database/Storage function"""
    if not session.get('logged_in'):
        return jsonify(success=False, message='Not authenticated'), 401
    
    try:
        # TODO: Add your CAN bus and Python actions here
        
        result = {
            'success': True,
            'message': 'Test Action 8 executed successfully',
            'action': 'database_storage',
            'timestamp': datetime.now().isoformat(),
            'data': {
                'status': 'completed',
                'details': 'Database/Storage function executed'
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