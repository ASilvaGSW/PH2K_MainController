from flask import Blueprint, request, jsonify, session
from datetime import datetime
import sys
import os
import time
import tkinter as tk
from tkinter import ttk
import threading

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

first_pick_after_align = False

# ======================== Pause / Stop Control ========================

_pause_event = threading.Event()
_pause_event.set()
_stop_requested = False


def wait_if_paused():
    """Blocks while paused. Returns False when stop has been requested."""
    _pause_event.wait()
    return not _stop_requested


# ======================== Machine Functions ========================

def prefeedHose():
    global operation_status, lubrication_feeder, insertion_servos

    if not wait_if_paused(): return "stopped"

    lubrication_feeder.close_hose_holder()
    insertion_servos.holder_hose_nozzle_semi_close()
    print(lubrication_feeder.move_feeder_until_ir(speed=290, wait=False))

    return "success"


#Move Pick and Place
def movePickandPlace():
    global pick_and_place, insertion_servos, insertion_jig, pick_and_place_camera, first_pick_after_align

    receiving_x = 6500
    receiving_z = 7000

    gap = -20
    zgap = 0

    #Nozzle Data
    nozzle_high = -965
    nozzle_x = [-580,-450,-300,-170,-20,110]
    trans_nozzle_high = -600

    deliver_nozzle_x = -3633
    deliver_nozzle_z = -1005

    # Joint Data
    joint_high = -965
    joint_x = [-1770,-1690,-1610,-1530,-1450,-1370,-1290,-1210,-1130,-1050]
    trans_joint_high = -600

    deliver_joint_x = -3919
    deliver_joint_z = -1005

    # Initial Setup
    if insertion_servos.slider_joint_receive() != "success": return "error01"
    if insertion_servos.slider_nozzle_receive() != "success": return "error02"
    if insertion_jig.move_z_axis(receiving_z) != "success": return "error03"
    if insertion_jig.move_x_axis(receiving_x) != "success": return "error04"
    if pick_and_place.open_gripper() != "success": return "error05"

    if not wait_if_paused(): return "stopped"

    # Validate Home
    if pick_and_place.move_actuator_z(0) != "success": return "error06"
    if pick_and_place.move_actuator_x(0) != "success": return "error07"

    # Pick Nozzle
    time.sleep(1)
    n_nozzle = pick_and_place_camera.pick_up_nozzle()
    print(n_nozzle)

    if n_nozzle == 255:
        print("Nozzle row empty, aligning...")
        pick_and_place_camera.custom_alignment_nozzle()
        first_pick_after_align = True
        n_nozzle = pick_and_place_camera.pick_up_nozzle()
        if n_nozzle == 255:
            return "error08"

    if n_nozzle is None:
        return "error08.1"

    if n_nozzle >= len(nozzle_x):
        n_nozzle = len(nozzle_x) - 1
        return f"error09_index_{n_nozzle}"

    if pick_and_place.move_actuator_x(nozzle_x[n_nozzle]- gap) != "success": return "error09"
    if pick_and_place.move_actuator_z(nozzle_high+zgap) != "success": return "error10"
    if pick_and_place.close_gripper() != "success": return "error11"

    if not wait_if_paused(): return "stopped"

    # Deliver Nozzle
    if pick_and_place.move_actuator_z(trans_nozzle_high) != "success": return "error12"
    if pick_and_place.move_actuator_x(deliver_nozzle_x - gap) != "success": return "error13"
    if pick_and_place.move_actuator_z(deliver_nozzle_z+zgap) != "success": return "error14"
    if pick_and_place.open_gripper() != "success": return "error15"
    time.sleep(.5)

    if not wait_if_paused(): return "stopped"

    # Joint Pick Up
    if pick_and_place.move_actuator_z(0) != "success": return "error16"

    n_joint = pick_and_place_camera.pick_up_joint()

    if n_joint == 255:
        print("Joint row empty, aligning...")
        pick_and_place_camera.custom_alignment_joint()
        first_pick_after_align = True
        n_joint = pick_and_place_camera.pick_up_joint()
        if n_joint == 255:
            return "error17"

    if n_joint is None:
        return "error17.1"

    if n_joint >= len(joint_x):
        return f"error21_index_{n_joint}"

    if pick_and_place.move_actuator_x(joint_x[n_joint]-gap) != "success": return "error18"
    if pick_and_place.move_actuator_z(joint_high+zgap) != "success": return "error20"
    if pick_and_place.close_gripper() != "success": return "error21"

    if not wait_if_paused(): return "stopped"

    # Deliver Joint
    if pick_and_place.move_actuator_z(trans_joint_high) != "success": return "error22"
    if pick_and_place.move_actuator_x(deliver_joint_x-gap) != "success": return "error23"
    if pick_and_place.move_actuator_z(deliver_joint_z+zgap) != "success": return "error24"
    if pick_and_place.open_gripper() != "success": return "error25"

    if not wait_if_paused(): return "stopped"

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

    if not wait_if_paused(): return "stopped"

    # Clamping and Reset Gripper
    if insertion_servos.clamp_nozzle_close() != "success" : return "error07"
    if insertion_servos.clamp_joint_close() != "success" : return "error08"
    if puller_extension.open_gripper() != "success" : return "error09"

    # Homing Slider Before Movement
    if insertion_servos.slider_joint_home() != "success" : return "error10"
    if insertion_servos.slider_nozzle_receive() != "success" : return "error11"

    # Insertio Jig Home POsition
    if hose_jig.insertion_position(False) != "success" : return "error12"

    if not wait_if_paused(): return "stopped"

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

    if not wait_if_paused(): return "stopped"

    # Go to Down Position for Hose Puller
    if insertion_jig.move_z_axis(insertion_jig_safe_zone) != "success" : return "error27"
    if insertion_jig.move_x_axis(lubricate_joint,flag=False) != "success" : return "6"

    # Preparing Hose Puller and Hose Jig
    if hose_puller.move_z_actuator(safe_position) != "success" : return "error29"
    if hose_puller.move_y_actuator(home_y) != "success" : return "error30"
    if hose_jig.insertion_position() != "success" : return "error31"

    if not wait_if_paused(): return "stopped"

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

    if not wait_if_paused(): return "stopped"

    # Alignment for Joint Insertion
    if hose_puller.move_y_actuator_with_speed(alignmnet_for_joint,200) != "success" : return "error13"
    if insertion_servos.holder_hose_joint_semi_close() != "success" : return "error42"
    if hose_puller.move_y_axis_until_no_hose(hose_puller_y_speed_for_alignment) != "success" : return "error43"
    if insertion_servos.holder_hose_joint_close() != "success" : return "error45"

    # Lubricate Hose on Joint Area
    if insertion_jig.move_x_axis(lubricate_joint) != "success" : return "6"
    if insertion_jig.move_z_axis(librication_position_joint_z) != "success" : return "error47"
    if lubrication_feeder.lubricate_joint(lubricate_joint_time) != "success" : return "error48"

    if not wait_if_paused(): return "stopped"

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

    if not wait_if_paused(): return "stopped"

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

    if hose_jig.gripper_close() != "success" : return "001"
    if transporter_fuyus.pickHome() != "success" : return "01"

    if not wait_if_paused(): return "stopped"

    if transporter_fuyus.moveToReceivingPosition() != "success" : return "03"
    if transporter_fuyus.pickHoseFromFirstStation() != "success" : return "04"
    if transporter_grippers.close_all_grippers() != "success" : return "05"
    time.sleep(1)
    if hose_jig.gripper_open() != "success" : return "06"

    if not wait_if_paused(): return "stopped"

    if transporter_fuyus.pickHome() != "success": return "07"
    if transporter_fuyus.moveToDeliverPosition() != "success": return "03"
    if transporter_grippers.open_all_grippers() != "success": return "05"
    if transporter_fuyus.pickHome() != "success": return "07"
    if transporter_fuyus.moveToSafeSpace() != "success": return "08"

    return "success"


# ======================== Tkinter UI ========================

STEPS = [
    ("prefeedHose",       "Pre-Feed Hose",    prefeedHose),
    ("movePickandPlace",  "Pick & Place",     movePickandPlace),
    ("oneCycle",          "Insertion Cycle",   oneCycle),
    ("pickUpHose",        "Transport Hose",    pickUpHose),
]

_C = {
    "bg":       "#0d1117",
    "panel":    "#161b22",
    "panel_hd": "#1c2128",
    "border":   "#30363d",
    "txt":      "#e6edf3",
    "txt2":     "#8b949e",
    "accent":   "#58a6ff",
    "green":    "#3fb950",
    "amber":    "#d29922",
    "red":      "#f85149",
    "cyan":     "#39d2c0",
    "dark":     "#010409",
}

_STEP_LABELS = ["1  PRE-FEED", "2  PICK & PLACE", "3  INSERTION", "4  TRANSPORT"]


class CycleRunnerApp:

    def __init__(self, root):
        self.root = root
        self.root.title("GSW - Hose Assembly System")
        self.root.configure(bg=_C["bg"])
        self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
        self.root.bind('<F11>', lambda e: self.root.attributes(
            '-fullscreen', not self.root.attributes('-fullscreen')))

        self.total_cycles = tk.IntVar(value=1)
        self.is_running = False

        self._lock = threading.Lock()
        self._current_cycle = 0
        self._current_step = 0
        self._cycle_times = []
        self._cycle_start = 0.0
        self._run_start = 0.0
        self._run_end = 0.0
        self._status_text = "READY"
        self._step_text = "---"
        self._finished = False

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick_clock()
        self._poll_ui()

    # ------------------------------------------------------------------ helpers

    def _panel(self, parent, title):
        outer = tk.Frame(parent, bg=_C["border"], padx=1, pady=1)
        hdr = tk.Frame(outer, bg=_C["panel_hd"])
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text=title, font=("Segoe UI", 9, "bold"),
                 bg=_C["panel_hd"], fg=_C["txt2"],
                 anchor=tk.W).pack(padx=10, pady=(4, 3))
        inner = tk.Frame(outer, bg=_C["panel"], padx=14, pady=10)
        inner.pack(fill=tk.BOTH, expand=True)
        return outer, inner

    @staticmethod
    def _fmt_time(secs):
        secs = max(0, secs)
        if secs >= 3600:
            return f"{int(secs//3600):02d}:{int(secs%3600//60):02d}:{int(secs%60):02d}"
        return f"{int(secs//60):02d}:{int(secs%60):02d}"

    # ------------------------------------------------------------------ build

    def _build_ui(self):
        self._build_header()
        tk.Frame(self.root, bg=_C["accent"], height=2).pack(fill=tk.X)

        body = tk.Frame(self.root, bg=_C["bg"], padx=28, pady=14)
        body.pack(fill=tk.BOTH, expand=True)

        top = tk.Frame(body, bg=_C["bg"])
        top.pack(fill=tk.BOTH, expand=True)
        top.columnconfigure(0, weight=1, uniform="half")
        top.columnconfigure(1, weight=1, uniform="half")

        self._build_left(top)
        self._build_right(top)
        self._build_progress(body)
        self._build_log(body)

    # --- header ---

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=_C["dark"], height=58)
        hdr.pack(fill=tk.X)
        hdr.pack_propagate(False)

        lf = tk.Frame(hdr, bg=_C["dark"])
        lf.pack(side=tk.LEFT, padx=24, fill=tk.Y)
        tk.Label(lf, text="GSW", font=("Segoe UI", 24, "bold"),
                 bg=_C["dark"], fg=_C["accent"]).pack(side=tk.LEFT)
        tk.Label(lf, text="HOSE ASSEMBLY SYSTEM",
                 font=("Segoe UI", 13), bg=_C["dark"],
                 fg=_C["txt"]).pack(side=tk.LEFT, padx=(14, 0), pady=(6, 0))

        rf = tk.Frame(hdr, bg=_C["dark"])
        rf.pack(side=tk.RIGHT, padx=24, fill=tk.Y)

        self.lbl_clock = tk.Label(rf, font=("Consolas", 13),
                                  bg=_C["dark"], fg=_C["txt2"])
        self.lbl_clock.pack(side=tk.RIGHT, pady=16)

        can_ok = canbus is not None
        can_c = _C["green"] if can_ok else _C["red"]
        can_t = "CAN  ONLINE" if can_ok else "CAN  OFFLINE"
        tk.Label(rf, text=can_t, font=("Consolas", 10, "bold"),
                 bg=_C["dark"], fg=can_c).pack(side=tk.RIGHT, padx=(0, 28), pady=18)

    # --- left column ---

    def _build_left(self, parent):
        col = tk.Frame(parent, bg=_C["bg"])
        col.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        po, pi = self._panel(col, "PROCESS SEQUENCE")
        po.pack(fill=tk.X, pady=(0, 10))
        self._build_steps(pi)

        co, ci = self._panel(col, "CYCLE CONFIGURATION")
        co.pack(fill=tk.X)
        self._build_config(ci)

    def _build_steps(self, parent):
        row = tk.Frame(parent, bg=_C["panel"])
        row.pack(fill=tk.X, pady=4)
        self.step_frames, self.step_lbls, self.step_dots = [], [], []

        for i, txt in enumerate(_STEP_LABELS):
            if i > 0:
                tk.Label(row, text="\u2192", font=("Segoe UI", 16),
                         bg=_C["panel"], fg=_C["txt2"]).pack(side=tk.LEFT, padx=6)

            border = tk.Frame(row, bg=_C["border"], padx=1, pady=1)
            border.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

            cell = tk.Frame(border, bg=_C["bg"], padx=10, pady=10)
            cell.pack(fill=tk.BOTH)

            dot = tk.Label(cell, text="\u25CF", font=("Segoe UI", 9),
                           bg=_C["bg"], fg=_C["txt2"])
            dot.pack(side=tk.LEFT, padx=(0, 6))

            lbl = tk.Label(cell, text=txt, font=("Consolas", 10, "bold"),
                           bg=_C["bg"], fg=_C["txt2"])
            lbl.pack(side=tk.LEFT)

            self.step_frames.append(cell)
            self.step_lbls.append(lbl)
            self.step_dots.append(dot)

    def _build_config(self, parent):
        top_row = tk.Frame(parent, bg=_C["panel"])
        top_row.pack(fill=tk.X, pady=(0, 14))

        tk.Label(top_row, text="TARGET CYCLES", font=("Segoe UI", 11),
                 bg=_C["panel"], fg=_C["txt2"]).pack(side=tk.LEFT)

        self.spin = tk.Spinbox(
            top_row, from_=1, to=9999, textvariable=self.total_cycles,
            width=6, font=("Consolas", 18, "bold"), justify=tk.CENTER,
            bg="#21262d", fg=_C["txt"], insertbackground=_C["txt"],
            highlightbackground=_C["border"], highlightthickness=1,
            buttonbackground=_C["panel_hd"], relief=tk.FLAT
        )
        self.spin.pack(side=tk.LEFT, padx=20)

        _bf = ("Segoe UI", 12, "bold")

        r1 = tk.Frame(parent, bg=_C["panel"])
        r1.pack(fill=tk.X)
        r2 = tk.Frame(parent, bg=_C["panel"])
        r2.pack(fill=tk.X, pady=(6, 0))

        self.btn_start = tk.Button(
            r1, text="\u25B6  START", command=self._on_start,
            bg="#238636", fg="white", activebackground="#2ea043",
            activeforeground="white", font=_bf, relief=tk.FLAT,
            cursor="hand2", width=14, height=2, bd=0)
        self.btn_start.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_pause = tk.Button(
            r1, text="\u23F8  PAUSE", command=self._on_pause,
            bg="#9e6a03", fg="white", activebackground="#bb8009",
            activeforeground="white", state=tk.DISABLED, font=_bf,
            relief=tk.FLAT, cursor="hand2", width=14, height=2, bd=0)
        self.btn_pause.pack(side=tk.LEFT, padx=8)

        self.btn_resume = tk.Button(
            r2, text="\u25B6  RESUME", command=self._on_resume,
            bg="#1f6feb", fg="white", activebackground="#388bfd",
            activeforeground="white", state=tk.DISABLED, font=_bf,
            relief=tk.FLAT, cursor="hand2", width=14, height=2, bd=0)
        self.btn_resume.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_stop = tk.Button(
            r2, text="\u23F9  STOP", command=self._on_stop,
            bg="#da3633", fg="white", activebackground="#f85149",
            activeforeground="white", state=tk.DISABLED, font=_bf,
            relief=tk.FLAT, cursor="hand2", width=14, height=2, bd=0)
        self.btn_stop.pack(side=tk.LEFT, padx=8)

    # --- right column ---

    def _build_right(self, parent):
        col = tk.Frame(parent, bg=_C["bg"])
        col.grid(row=0, column=1, sticky="nsew", padx=(10, 0))

        so, si = self._panel(col, "MACHINE STATUS")
        so.pack(fill=tk.X, pady=(0, 10))
        self._build_status(si)

        to, ti = self._panel(col, "TIME ESTIMATION")
        to.pack(fill=tk.X)
        self._build_time(ti)

    def _build_status(self, parent):
        def _row(label_text):
            r = tk.Frame(parent, bg=_C["panel"])
            r.pack(fill=tk.X, pady=5)
            tk.Label(r, text=label_text, font=("Segoe UI", 11),
                     bg=_C["panel"], fg=_C["txt2"], width=14,
                     anchor=tk.W).pack(side=tk.LEFT)
            return r

        sr = _row("STATUS")
        self.status_dot = tk.Label(sr, text="\u25CF", font=("Segoe UI", 16),
                                   bg=_C["panel"], fg=_C["txt2"])
        self.status_dot.pack(side=tk.LEFT, padx=(0, 8))
        self.lbl_status = tk.Label(sr, text="READY",
                                   font=("Consolas", 16, "bold"),
                                   bg=_C["panel"], fg=_C["txt2"])
        self.lbl_status.pack(side=tk.LEFT)

        cr = _row("CYCLE")
        self.lbl_cycle = tk.Label(cr, text="0 / 0",
                                  font=("Consolas", 16, "bold"),
                                  bg=_C["panel"], fg=_C["txt"])
        self.lbl_cycle.pack(side=tk.LEFT)

        st = _row("CURRENT STEP")
        self.lbl_step = tk.Label(st, text="---",
                                 font=("Consolas", 13),
                                 bg=_C["panel"], fg=_C["txt"])
        self.lbl_step.pack(side=tk.LEFT)

    def _build_time(self, parent):
        def _time_row(label_text, default):
            r = tk.Frame(parent, bg=_C["panel"])
            r.pack(fill=tk.X, pady=5)
            tk.Label(r, text=label_text, font=("Segoe UI", 11),
                     bg=_C["panel"], fg=_C["txt2"], width=14,
                     anchor=tk.W).pack(side=tk.LEFT)
            lbl = tk.Label(r, text=default, font=("Consolas", 16, "bold"),
                           bg=_C["panel"], fg=_C["txt"])
            lbl.pack(side=tk.LEFT)
            return lbl

        self.lbl_remaining = _time_row("REMAINING", "--:--")
        self.lbl_avg = _time_row("AVG CYCLE", "---")
        self.lbl_elapsed = _time_row("ELAPSED", "00:00")

    # --- progress ---

    def _build_progress(self, parent):
        po, pi = self._panel(parent, "OVERALL PROGRESS")
        po.pack(fill=tk.X, pady=(12, 10))

        self._pbar = tk.Canvas(pi, height=38, bg=_C["panel"], highlightthickness=0)
        self._pbar.pack(fill=tk.X, pady=(0, 2))
        self._pbar.bind('<Configure>', lambda e: self._draw_bar())
        self._pval = 0.0

    def _draw_bar(self):
        c = self._pbar
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 20:
            return
        c.create_rectangle(0, 0, w, h, fill="#21262d", outline=_C["border"])
        fw = max(0, w * self._pval / 100)
        if fw > 0:
            fill = _C["green"] if self._pval >= 100 else _C["accent"]
            c.create_rectangle(0, 0, fw, h, fill=fill, outline="")
        c.create_text(w // 2, h // 2, text=f"{self._pval:.1f} %",
                      fill="white", font=("Consolas", 15, "bold"))

    # --- log ---

    def _build_log(self, parent):
        lo, li = self._panel(parent, "SYSTEM LOG")
        lo.pack(fill=tk.BOTH, expand=True)

        frame = tk.Frame(li, bg=_C["panel"])
        frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(
            frame, font=("Consolas", 10), state=tk.DISABLED, wrap=tk.WORD,
            bg="#0d1117", fg="#8b949e", insertbackground=_C["txt"],
            selectbackground="#264f78", highlightthickness=1,
            highlightbackground=_C["border"], relief=tk.FLAT)
        sb = tk.Scrollbar(frame, orient=tk.VERTICAL, command=self.log_text.yview,
                          bg=_C["panel"], troughcolor="#21262d")
        self.log_text.configure(yscrollcommand=sb.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

    # ------------------------------------------------------------------ clock

    def _tick_clock(self):
        self.lbl_clock.config(text=datetime.now().strftime("%b %d, %Y   %H:%M:%S"))
        self.root.after(1000, self._tick_clock)

    # ------------------------------------------------------------------ log

    def _log(self, msg):
        stamp = datetime.now().strftime("%H:%M:%S")
        def _do():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"  [{stamp}]  {msg}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, _do)

    # ------------------------------------------------------------------ step indicators

    def _color_steps(self, active):
        for i, (f, l, d) in enumerate(zip(self.step_frames, self.step_lbls, self.step_dots)):
            if i < active:
                c, bg = _C["green"], _C["bg"]
            elif i == active:
                c, bg = _C["cyan"], "#0d2137"
            else:
                c, bg = _C["txt2"], _C["bg"]
            f.configure(bg=bg)
            l.configure(fg=c, bg=bg)
            d.configure(fg=c, bg=bg)

    def _reset_steps(self):
        for f, l, d in zip(self.step_frames, self.step_lbls, self.step_dots):
            f.configure(bg=_C["bg"])
            l.configure(fg=_C["txt2"], bg=_C["bg"])
            d.configure(fg=_C["txt2"], bg=_C["bg"])

    # ------------------------------------------------------------------ poll UI

    def _poll_ui(self):
        with self._lock:
            cyc        = self._current_cycle
            step       = self._current_step
            status     = self._status_text
            step_text  = self._step_text
            ctimes     = list(self._cycle_times)
            cstart     = self._cycle_start
            rstart     = self._run_start
            rend       = self._run_end
            finished   = self._finished

        color_map = {"READY": _C["txt2"], "RUNNING": _C["green"],
                     "PAUSED": _C["amber"], "STOPPING": _C["amber"],
                     "STOPPED": _C["red"], "COMPLETED": _C["green"],
                     "FAULT": _C["red"]}
        sc = _C["txt"]
        for k, v in color_map.items():
            if k in status.upper():
                sc = v
                break

        self.lbl_status.config(text=status, fg=sc)
        self.status_dot.config(fg=sc)
        self.lbl_step.config(text=step_text)

        if self.is_running or finished:
            total = self.total_cycles.get()
            self.lbl_cycle.config(text=f"{cyc} / {total}")

            if self.is_running:
                self._color_steps(step)
            elif finished:
                self._color_steps(4)

            ts = total * 4
            ds = (cyc - 1) * 4 + step if cyc > 0 else 0
            if finished:
                ds = ts
            pct = (ds / ts) * 100 if ts > 0 else 0
            self._pval = pct
            self._draw_bar()

            if rstart > 0:
                end = rend if rend > 0 else time.time()
                self.lbl_elapsed.config(text=self._fmt_time(end - rstart))

            if ctimes:
                avg = sum(ctimes) / len(ctimes)
                cc = len(ctimes)
                if cstart > 0:
                    rem = max(0, avg - (time.time() - cstart)) + max(0, total - cc - 1) * avg
                else:
                    rem = (total - cc) * avg
                self.lbl_remaining.config(text=self._fmt_time(rem))
                self.lbl_avg.config(text=f"{avg:.1f}s")
            elif self.is_running:
                self.lbl_remaining.config(text="Estimating...")
                self.lbl_avg.config(text="---")

        self.root.after(250, self._poll_ui)

    # ------------------------------------------------------------------ actions

    def _on_start(self):
        global _stop_requested, _pause_event
        try:
            n = self.total_cycles.get()
        except tk.TclError:
            self._log("ERROR: Enter a valid number of cycles")
            return
        if n <= 0:
            self._log("ERROR: Number of cycles must be greater than 0")
            return
        if canbus is None:
            self._log("ERROR: CAN bus unavailable - check hardware and drivers")
            return

        _stop_requested = False
        _pause_event.set()
        self.is_running = True

        with self._lock:
            self._current_cycle = 0
            self._current_step = 0
            self._cycle_times = []
            self._cycle_start = 0.0
            self._run_start = time.time()
            self._run_end = 0.0
            self._status_text = "RUNNING"
            self._step_text = "---"
            self._finished = False

        self._reset_steps()
        self.btn_start.config(state=tk.DISABLED)
        self.spin.config(state=tk.DISABLED)
        self.btn_pause.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.NORMAL)
        self.btn_resume.config(state=tk.DISABLED)
        threading.Thread(target=self._run_cycles, daemon=True).start()

    def _on_pause(self):
        _pause_event.clear()
        self.btn_pause.config(state=tk.DISABLED)
        self.btn_resume.config(state=tk.NORMAL)
        with self._lock:
            self._status_text = "PAUSED"
        self._log("System paused by operator")

    def _on_resume(self):
        _pause_event.set()
        self.btn_pause.config(state=tk.NORMAL)
        self.btn_resume.config(state=tk.DISABLED)
        with self._lock:
            self._status_text = "RUNNING"
        self._log("System resumed")

    def _on_stop(self):
        global _stop_requested
        _stop_requested = True
        _pause_event.set()
        with self._lock:
            self._status_text = "STOPPING..."
        self._log("Stop requested - completing current operation...")

    # ------------------------------------------------------------------ worker

    def _run_cycles(self):
        total = self.total_cycles.get()
        self._log(f"Production run started: {total} cycle(s)")

        for i in range(total):
            cnum = i + 1
            cstart = time.time()
            with self._lock:
                self._current_cycle = cnum
                self._cycle_start = cstart

            self._log(f"── Cycle {cnum}/{total} ──")

            for si, (fname, dname, func) in enumerate(STEPS):
                if _stop_requested:
                    self._log(f"Stopped before {dname}")
                    self._finish("STOPPED BY USER")
                    return

                with self._lock:
                    self._current_step = si
                    self._step_text = dname
                    self._status_text = f"RUNNING: {dname}"

                self._log(f"    {dname}...")
                result = func()

                if result == "stopped":
                    self._log(f"    STOPPED during {dname}")
                    self._finish("STOPPED BY USER")
                    return
                if result != "success":
                    self._log(f"    FAULT in {dname}: {result}")
                    self._finish(f"FAULT: {result}")
                    return

                self._log(f"    {dname}: OK")

            ct = time.time() - cstart
            with self._lock:
                self._cycle_times.append(ct)
                self._cycle_start = 0.0
            self._log(f"Cycle {cnum} completed in {ct:.1f}s")

        with self._lock:
            self._finished = True
        self._log(f"Production run completed: {total} cycle(s) finished successfully")
        self._finish("COMPLETED")

    def _finish(self, final):
        with self._lock:
            self._status_text = final
            self._step_text = "---"
            self._run_end = time.time()
        self.is_running = False
        self.root.after(0, self._reset_buttons)

    def _reset_buttons(self):
        self.btn_start.config(state=tk.NORMAL)
        self.spin.config(state=tk.NORMAL)
        self.btn_pause.config(state=tk.DISABLED)
        self.btn_resume.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.DISABLED)

    def _on_close(self):
        global _stop_requested
        _stop_requested = True
        _pause_event.set()
        self.root.destroy()


def main():
    root = tk.Tk()
    CycleRunnerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
