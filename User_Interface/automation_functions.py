import time

# Global variables that need to be initialized from the main application
insertion_jig = None
insertion_servos = None
hose_puller = None
hose_jig = None
puller_extension = None
pick_and_place = None
lubrication_feeder = None
elevator_in = None

def initialize_devices(devices_dict):
    """
    Initialize all device instances from the main application.
    
    Args:
        devices_dict: Dictionary containing all device instances
                     Expected keys: 'insertion_jig', 'insertion_servos', 'hose_puller',
                                   'hose_jig', 'puller_extension', 'pick_and_place',
                                   'lubrication_feeder', 'elevator_in'
    """
    global insertion_jig, insertion_servos, hose_puller, hose_jig, puller_extension
    global pick_and_place, lubrication_feeder, elevator_in
    
    insertion_jig = devices_dict.get('insertion_jig')
    insertion_servos = devices_dict.get('insertion_servos')
    hose_puller = devices_dict.get('hose_puller')
    hose_jig = devices_dict.get('hose_jig')
    puller_extension = devices_dict.get('puller_extension')
    pick_and_place = devices_dict.get('pick_and_place')
    lubrication_feeder = devices_dict.get('lubrication_feeder')
    elevator_in = devices_dict.get('elevator_in')

def oneCycle(custom_params=None):
    """
    Execute one complete cycle of the hose assembly process.
    
    Args:
        custom_params (dict): Optional dictionary with custom parameters
    
    Returns:
        str: "success" if completed successfully, error code if failed
    """
    global insertion_jig, insertion_servos, hose_puller, hose_jig, puller_extension, pick_and_place, lubrication_feeder

    # Default parameters
    default_params = {
        'offset_x': 0,
        'offset_z': 0,
        'home_position_z': 4000,
        'home_position_x': 0,
        'lubrication_position_z': 500,
        'lubricate_nozzle': -5380,
        'insertion_position_z': 490,
        'insert_nozzle': -6570,
        'librication_position_joint_z': 580,
        'lubricate_joint': -11000,
        'insertion_position_joint_z': 600,
        'insert_joint': -8950,
        'safe_position': 200,
        'home_y': 4200,
        'wait_y': 5930,
        'pickup_y': 8990,
        'z_home': 50,
        'feeding_time': 1.53,
        'lubrication_time_nozzle': 5,
        'insertion_safe_position': 4000,
        'prefeeder_speed': 50,
        'pick_up_gap': 30,
        'after_pickup': 500,
        'position_to_second_rise': 1900,
        'second_rise': 25,
        'joint_pull': 687
    }
    
    # Merge custom parameters with defaults
    if custom_params:
        params = {**default_params, **custom_params}
    else:
        params = default_params

    #****************************** Insertion Jig ******************************

    offset_x = params['offset_x']
    offset_z = params['offset_z']

    home_position_z = params['home_position_z'] + offset_z
    home_position_x = params['home_position_x'] + offset_x

    lubrication_position_z = params['lubrication_position_z'] + offset_z
    lubricate_nozzle = params['lubricate_nozzle'] + offset_x

    insertion_position_z = params['insertion_position_z'] + offset_z
    insert_nozzle = params['insert_nozzle'] + offset_x  

    librication_position_joint_z = params['librication_position_joint_z'] + offset_z
    lubricate_joint = params['lubricate_joint'] + offset_x

    insertion_position_joint_z = params['insertion_position_joint_z'] + offset_z
    insert_joint = params['insert_joint'] + offset_x

    safe_position = params['safe_position']
    home_y = params['home_y']
    wait_y = params['wait_y']
    pickup_y = params['pickup_y']
    z_home = params['z_home']
    
    # Additional parameters
    feeding_time = params['feeding_time']
    lubrication_time_nozzle = params['lubrication_time_nozzle']
    insertion_safe_position = params['insertion_safe_position']
    prefeeder_speed = params['prefeeder_speed']
    pick_up_gap = params['pick_up_gap']
    after_pickup = params['after_pickup']
    position_to_second_rise = params['position_to_second_rise']
    second_rise = params['second_rise']
    joint_pull = params['joint_pull']

    #****************************** Routine ******************************

    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    if lubrication_feeder.feed_hose(duration=feeding_time) != "success" : return "error02"
    time.sleep(2)
    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error08"
    if lubrication_feeder.open_hose_holder() != "success" : return "error03"

    if insertion_servos.clamp_nozzle_close() != "success" : return "error08"
    if insertion_servos.clamp_joint_close() != "success" : return "error19"
    if puller_extension.open_gripper() != "success" : return "error20"

    if insertion_servos.slider_joint_home() != "success" : return "error08"
    if insertion_servos.slider_nozzle_home() != "success" : return "error08"

    #Nozzle Position

    if hose_jig.insertion_position(False) != "success" : return "error05"

    if insertion_jig.move_z_axis(home_position_z) != "success" : return "error01"
    if insertion_jig.move_x_axis(home_position_x) != "success" : return "error02"

    if insertion_servos.holder_hose_nozzle_close() != "success" : return "error18"

    if insertion_jig.move_z_axis(lubrication_position_z) != "success" : return "error03"
    if insertion_jig.move_x_axis(lubricate_nozzle) != "success" : return "error04"

    if lubrication_feeder.lubricate_nozzle(lubrication_time_nozzle) != "success" : return "error03.1"
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

    if insertion_jig.move_z_axis(insertion_safe_position) != "success" : return "error13"

    #Hose Puller Action

    if lubrication_feeder.move_pre_feeder(prefeeder_speed) != "success" : return "error04"

    if hose_puller.move_z_actuator(safe_position) != "success" : return "error04"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error03"
    if hose_jig.insertion_position() != "success" : return "error05"
    if hose_puller.move_y_actuator(pickup_y+pick_up_gap) != "success" : return "error06"
    if hose_puller.move_z_actuator(z_home) != "success" : return "error07"
    if puller_extension.close_gripper() != "success" : return "error08"
    if hose_puller.move_y_actuator(pickup_y-after_pickup) != "success" : return "error09"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error10"
    if hose_puller.move_y_actuator(wait_y+position_to_second_rise) != "success" : return "error11"
    if hose_puller.move_z_actuator(safe_position+second_rise) != "success" : return "error12"
    if hose_puller.move_y_actuator(wait_y) != "success" : return "error11"
    if lubrication_feeder.close_hose_holder() != "success" : return "error01"
    time.sleep(.5)
    if insertion_servos.activate_cutter() != "success" : return "error12"
    if hose_puller.move_y_actuator(home_y+joint_pull) != "success" : return "error13"
    
    if lubrication_feeder.move_pre_feeder(0) != "success" : return "error04"

    #Moving to Joint

    if insertion_servos.holder_hose_joint_close() != "success" : return "error18"

    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "error15"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error14"

    if lubrication_feeder.lubricate_joint(lubrication_time_joint) != "success" : return "error14.1"
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

    if insertion_jig.move_z_axis(home_position_z,False) != "success" : return "error24"
    if insertion_jig.move_x_axis(home_position_x,False) != "success" : return "error25"

    if hose_puller.move_z_actuator(z_home) != "success" : return "error14"
    if hose_jig.gripper_close() != "success" : return "error15"
    if puller_extension.open_gripper() != "success" : return "error16"
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error17"
    if hose_jig.deliver_position() != "success" : return "error18"
    if hose_jig.gripper_open() != "success" : return "error19"
    if hose_puller.move_z_actuator(0) != "success" : return "error20"

    return "success"

def testHome():
    """
    Test function for homing all devices and servos.
    
    Returns:
        str: "success" if completed successfully
    """
    global elevator_in, pick_and_place, hose_puller, hose_jig, insertion_jig, insertion_servos, puller_extension
    
    print("Homing Servos")

    print("Insertion Jig Servos")

    result = insertion_servos.cutter_open()
    print(f"Cutter Result: {result}")
    result = insertion_servos.holder_hose_nozzle_open()
    print(f"Holder Hose Nozzle Result: {result}")
    result = insertion_servos.clamp_nozzle_open()
    print(f"Clamp Nozzle Result: {result}")
    result = insertion_servos.slider_nozzle_home()
    print(f"Slider Nozzle Result: {result}")
    result = insertion_servos.holder_hose_joint_open()
    print(f"Holder Hose Joint Result: {result}")
    result = insertion_servos.clamp_joint_open()
    print(f"Clamp Joint Result: {result}")
    result = insertion_servos.slider_joint_home()
    print(f"Slider Joint Result: {result}")

    print("Grippers")

    result = pick_and_place.open_gripper()
    print(f"Gripper P&P Result: {result}")

    result = puller_extension.open_gripper()
    print(f"Gripper Puller Result: {result}")

    print("Calibration functions...")
    
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

    # Test PickAndPlace homing functions
    print("\n--- Testing PickAndPlace homing functions ---")
    
    print("Testing home_x_axis...")
    result = pick_and_place.home_x_axis()
    print(f"Result: {result}")
    
    print("Testing home_z_axis...")
    result = pick_and_place.home_z_axis()
    print(f"Result: {result}")

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
    
    print("\nAll homing tests completed!")

    return "success"