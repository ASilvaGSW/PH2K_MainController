"""
Cycle Parameter Tester - Aplicación de escritorio para configurar y probar parámetros de ciclo

Esta aplicación Tkinter permite:
- Configurar parámetros del ciclo de producción
- Conectar y probar hardware individual
- Ejecutar ciclos completos de prueba
- Guardar/cargar configuraciones de parámetros
- Monitorear logs en tiempo real

Funciones principales del ciclo completo implementadas:
1. Alimentación de manguera (feed_hose)
2. Cierre del sujetador de manguera (close_hose_holder)
3. Movimiento del jig de inserción a posición home
4. Lubricación de boquilla (lubricate_nozzle)
5. Inserción de boquilla completa con servos
6. Preparación del hose puller y hose jig
7. Acción de tirado de manguera (pulling action)
8. Corte de manguera (activate_cutter)
9. Alineación para inserción de junta
10. Lubricación del área de junta (lubricate_joint)
11. Inserción de junta completa con servos
12. Finalización y entrega de manguera

Clases de hardware integradas:
- Canbus: Comunicación CAN bus
- HoseJig: Control del jig de manguera
- HosePuller: Control del tirador de manguera
- PullerExtension: Extensión del tirador
- InsertionJig: Jig de inserción X/Z
- InsertionServos: Servos de inserción
- LubricationFeeder: Sistema de lubricación y alimentación

Autor: Sistema de Control PH2K
Versión: 1.0
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import json
import os
import sys
import time
import threading
import platform

# Add the main_controller directory to the path to import classes
sys.path.append(os.path.join(os.path.dirname(__file__), 'main_controller'))

# Import the required classes
if platform.system() == 'Linux':
    try:
        from classes.canbus_jetson import Canbus
        print("Usando implementación de Canbus para Jetson Orin Nano")
    except ImportError as e:
        print(f"Error al importar canbus_jetson: {e}")
        sys.exit(1)
else:
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
from classes.insertion_servos import InsertionServos
from classes.lubrication_feeder import LubricationFeeder
from classes.elevator_in import ElevatorIn
from classes.pick_and_place import PickAndPlace
from classes.transporter_fuyus import TransporterFuyus

class CycleParameterTester:
    def __init__(self, root):
        self.root = root
        self.root.title("Cycle Parameter Tester - PH2K")
        self.root.geometry("1200x800")
        
        # Initialize hardware classes
        self.canbus = None
        self.insertion_jig = None
        self.insertion_servos = None
        self.hose_puller = None
        self.hose_jig = None
        self.puller_extension = None
        self.lubrication_feeder = None
        self.elevator_in = None
        self.pick_and_place = None
        self.transporter_fuyus = None
        
        # Default parameters
        self.default_params = {
            # Insertion Jig Data
            'offset_x': 0,
            'offset_z': 0,
            'home_position_z': 4200,
            'home_position_x': 0,
            'lubrication_position_z': 520,
            'lubricate_nozzle': -5080,
            'insertion_position_z': 498,
            'insert_nozzle': -6570,
            'librication_position_joint_z': 530,
            'lubricate_joint': -11170,
            'insertion_position_joint_z': 600,
            'insert_joint': -8950,
            
            # Hose Puller Data
            'safe_position': 200,
            'safe_position_over_hose_jig': 242,
            'home_y': 4200,
            'wait_y': 5930,
            'cutting_position': 7830,
            'pickup_y': 9005,
            'before_rise_position': 8510,
            'z_home': 50,
            'z_picking_position': 55,
            'alignmnet_for_joint': 4865,
            
            # Custom Variables
            'insertion_jig_safe_zone': 4000,
            'preefeder_speed': 50,
            'feed_hose_time': 3.15,
            'lubricate_nozzle_time': 5,
            'lubricate_joint_time': 5,
            'hose_puller_y_speed': 200,
            'hose_puller_y_speed_for_alignment': 100,
            
            # Insertion Servo Positions
            'slider_nozzle_insertion_position': 0,
            'slider_joint_insertion_position': 10,
            
            # Component Placement Positions (initialized equal to home positions)
            'slider_nozzle_component_placement_position': 0,
            'slider_joint_component_placement_position': 0
        }
        
        self.current_params = self.default_params.copy()
        self.param_vars = {}
        
        self.create_widgets()
        self.load_parameters()
        
    def create_widgets(self):
        # Create main frame with scrollbar
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create notebook for tabs
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # Parameters tab
        params_frame = ttk.Frame(notebook)
        notebook.add(params_frame, text="Parámetros del Ciclo")
        
        # Control tab
        control_frame = ttk.Frame(notebook)
        notebook.add(control_frame, text="Control y Pruebas")
        
        # Create parameters interface
        self.create_parameters_tab(params_frame)
        
        # Create control interface
        self.create_control_tab(control_frame)
        
    def create_parameters_tab(self, parent):
        # Create scrollable frame
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Insertion Jig Parameters
        jig_frame = ttk.LabelFrame(scrollable_frame, text="Parámetros Insertion Jig", padding=10)
        jig_frame.pack(fill=tk.X, padx=5, pady=5)
        
        jig_params = [
            ('offset_x', 'Offset X'),
            ('offset_z', 'Offset Z'),
            ('home_position_z', 'Home Position Z'),
            ('home_position_x', 'Home Position X'),
            ('lubrication_position_z', 'Lubrication Position Z'),
            ('lubricate_nozzle', 'Lubricate Nozzle'),
            ('insertion_position_z', 'Insertion Position Z'),
            ('insert_nozzle', 'Insert Nozzle'),
            ('librication_position_joint_z', 'Lubrication Position Joint Z'),
            ('lubricate_joint', 'Lubricate Joint'),
            ('insertion_position_joint_z', 'Insertion Position Joint Z'),
            ('insert_joint', 'Insert Joint')
        ]
        
        for i, (param, label) in enumerate(jig_params):
            row = i // 2
            col = (i % 2) * 2
            
            ttk.Label(jig_frame, text=f"{label}:").grid(row=row, column=col, sticky=tk.W, padx=5, pady=2)
            var = tk.StringVar(value=str(self.current_params[param]))
            self.param_vars[param] = var
            entry = ttk.Entry(jig_frame, textvariable=var, width=15)
            entry.grid(row=row, column=col+1, padx=5, pady=2)
        
        # Hose Puller Parameters
        puller_frame = ttk.LabelFrame(scrollable_frame, text="Parámetros Hose Puller", padding=10)
        puller_frame.pack(fill=tk.X, padx=5, pady=5)
        
        puller_params = [
            ('safe_position', 'Safe Position'),
            ('safe_position_over_hose_jig', 'Safe Position Over Hose Jig'),
            ('home_y', 'Home Y'),
            ('wait_y', 'Wait Y'),
            ('cutting_position', 'Cutting Position'),
            ('pickup_y', 'Pickup Y'),
            ('before_rise_position', 'Before Rise Position'),
            ('z_home', 'Z Home'),
            ('z_picking_position', 'Z Picking Position'),
            ('alignmnet_for_joint', 'Alignment for Joint')
        ]
        
        for i, (param, label) in enumerate(puller_params):
            row = i // 2
            col = (i % 2) * 2
            
            ttk.Label(puller_frame, text=f"{label}:").grid(row=row, column=col, sticky=tk.W, padx=5, pady=2)
            var = tk.StringVar(value=str(self.current_params[param]))
            self.param_vars[param] = var
            entry = ttk.Entry(puller_frame, textvariable=var, width=15)
            entry.grid(row=row, column=col+1, padx=5, pady=2)
        
        # Custom Variables
        custom_frame = ttk.LabelFrame(scrollable_frame, text="Variables Personalizadas", padding=10)
        custom_frame.pack(fill=tk.X, padx=5, pady=5)
        
        custom_params = [
            ('insertion_jig_safe_zone', 'Insertion Jig Safe Zone'),
            ('preefeder_speed', 'Prefeeder Speed'),
            ('feed_hose_time', 'Feed Hose Time'),
            ('lubricate_nozzle_time', 'Lubricate Nozzle Time'),
            ('lubricate_joint_time', 'Lubricate Joint Time'),
            ('hose_puller_y_speed', 'Hose Puller Y Speed'),
            ('hose_puller_y_speed_for_alignment', 'Hose Puller Y Speed for Alignment'),
            ('slider_nozzle_insertion_position', 'Slider Nozzle Insertion Position'),
            ('slider_joint_insertion_position', 'Slider Joint Insertion Position'),
            ('slider_nozzle_component_placement_position', 'Slider Nozzle Component Placement Position'),
            ('slider_joint_component_placement_position', 'Slider Joint Component Placement Position')
        ]
        
        for i, (param, label) in enumerate(custom_params):
            row = i // 2
            col = (i % 2) * 2
            
            ttk.Label(custom_frame, text=f"{label}:").grid(row=row, column=col, sticky=tk.W, padx=5, pady=2)
            var = tk.StringVar(value=str(self.current_params[param]))
            self.param_vars[param] = var
            entry = ttk.Entry(custom_frame, textvariable=var, width=15)
            entry.grid(row=row, column=col+1, padx=5, pady=2)
        
        # Buttons frame
        buttons_frame = ttk.Frame(scrollable_frame)
        buttons_frame.pack(fill=tk.X, padx=5, pady=10)
        
        ttk.Button(buttons_frame, text="Guardar Parámetros", command=self.save_parameters).pack(side=tk.LEFT, padx=5)
        ttk.Button(buttons_frame, text="Cargar Parámetros", command=self.load_parameters).pack(side=tk.LEFT, padx=5)
        ttk.Button(buttons_frame, text="Restaurar Valores por Defecto", command=self.reset_to_defaults).pack(side=tk.LEFT, padx=5)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
    def create_control_tab(self, parent):
        # Connection frame
        conn_frame = ttk.LabelFrame(parent, text="Conexión Hardware", padding=10)
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.connection_status = tk.StringVar(value="Desconectado")
        ttk.Label(conn_frame, text="Estado:").pack(side=tk.LEFT, padx=5)
        ttk.Label(conn_frame, textvariable=self.connection_status, foreground="red").pack(side=tk.LEFT, padx=5)
        
        ttk.Button(conn_frame, text="Conectar Hardware", command=self.connect_hardware).pack(side=tk.RIGHT, padx=5)
        ttk.Button(conn_frame, text="Desconectar", command=self.disconnect_hardware).pack(side=tk.RIGHT, padx=5)
        
        # Test controls frame
        test_frame = ttk.LabelFrame(parent, text="Controles de Prueba", padding=10)
        test_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(test_frame, text="Ejecutar Ciclo Completo", command=self.run_full_cycle, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        ttk.Button(test_frame, text="Probar Movimientos Individuales", command=self.test_individual_movements, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        ttk.Button(test_frame, text="Enviar Todo a Home", command=self.send_all_to_home, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        ttk.Button(test_frame, text="Detener Prueba", command=self.stop_test, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        
        # Log frame
        log_frame = ttk.LabelFrame(parent, text="Log de Pruebas", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=20, width=80)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Clear log button
        ttk.Button(log_frame, text="Limpiar Log", command=self.clear_log).pack(pady=5)
        
    def log_message(self, message):
        """Add message to log with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update()
        
    def clear_log(self):
        """Clear the log text"""
        self.log_text.delete(1.0, tk.END)
        
    def save_parameters(self):
        """Save current parameters to JSON file with validation"""
        try:
            # Validate parameters before saving
            if not self.validate_parameters():
                return
                
            # Update current_params with values from GUI
            for param, var in self.param_vars.items():
                try:
                    value = float(var.get()) if '.' in var.get() else int(var.get())
                    self.current_params[param] = value
                except ValueError:
                    messagebox.showerror("Error", f"Valor inválido para {param}: {var.get()}")
                    return
            
            # Save to file
            with open('cycle_parameters.json', 'w') as f:
                json.dump(self.current_params, f, indent=4)
            
            messagebox.showinfo("Éxito", "Parámetros guardados correctamente")
            self.log_message("Parámetros guardados en cycle_parameters.json")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar parámetros: {str(e)}")
    
    def validate_parameters(self):
        """Validate that all numeric parameters are valid"""
        try:
            for param, var in self.param_vars.items():
                value_str = var.get().strip()
                if not value_str:
                    messagebox.showerror("Error de Validación", f"El parámetro '{param}' no puede estar vacío")
                    return False
                
                try:
                    float_value = float(value_str)
                    
                    # Specific validations by parameter type
                    if 'speed' in param.lower() and float_value < 0:
                        messagebox.showerror("Error de Validación", f"La velocidad '{param}' no puede ser negativa")
                        return False
                    
                    if 'time' in param.lower() and float_value < 0:
                        messagebox.showerror("Error de Validación", f"El tiempo '{param}' no puede ser negativo")
                        return False
                    
                    if 'position' in param.lower() and abs(float_value) > 10000:
                        messagebox.showerror("Error de Validación", f"La posición '{param}' está fuera del rango válido (-10000 a 10000)")
                        return False
                    
                    # Additional specific validations
                    if param == 'preefeder_speed' and (float_value < 0 or float_value > 100):
                        messagebox.showerror("Error de Validación", f"La velocidad del prefeeder debe estar entre 0 y 100")
                        return False
                    
                    if 'lubricate' in param.lower() and 'time' in param.lower() and (float_value < 0 or float_value > 10):
                        messagebox.showerror("Error de Validación", f"El tiempo de lubricación '{param}' debe estar entre 0 y 10 segundos")
                        return False
                        
                except ValueError:
                    messagebox.showerror("Error de Validación", f"El parámetro '{param}' debe ser un número válido")
                    return False
            
            return True
            
        except Exception as e:
            messagebox.showerror("Error", f"Error durante la validación: {str(e)}")
            return False
            
    def load_parameters(self):
        """Load parameters from JSON file"""
        try:
            if os.path.exists('cycle_parameters.json'):
                with open('cycle_parameters.json', 'r') as f:
                    loaded_params = json.load(f)
                
                # Update current params and GUI
                self.current_params.update(loaded_params)
                for param, value in self.current_params.items():
                    if param in self.param_vars:
                        self.param_vars[param].set(str(value))
                
                self.log_message("Parámetros cargados desde cycle_parameters.json")
            else:
                self.log_message("No se encontró archivo de parámetros, usando valores por defecto")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al cargar parámetros: {str(e)}")
            
    def reset_to_defaults(self):
        """Reset all parameters to default values"""
        self.current_params = self.default_params.copy()
        for param, value in self.current_params.items():
            if param in self.param_vars:
                self.param_vars[param].set(str(value))
        
        self.log_message("Parámetros restaurados a valores por defecto")
        
    def connect_hardware(self):
        """Initialize hardware connections"""
        try:
            self.log_message("Iniciando conexión con hardware...")
            
            # Initialize CAN bus
            self.canbus = Canbus()
            start_result = self.canbus.start_canbus()
            if not start_result:
                raise Exception("Error al inicializar CAN bus")
            
            # Initialize hardware classes with CAN bus IDs
            CANBUS_ID_JIG = 0x0CA
            CANBUS_ID_PULLER = 0x192
            CANBUS_ID_EXTENSION = 0x193
            CANBUS_ID_INSERTION = 0x0C9
            CANBUS_ID_INSERTION_SERVOS = 0x002
            CANBUS_ID_LUBRICATION_FEEDER = 0x019
            CANBUS_ID_ELEVATOR_IN = 0x189
            CANBUS_ID_PICK_AND_PLACE = 0x191
            CANBUS_ID_TRANSPORTER_FUYUS = 0x021
            
            self.insertion_jig = InsertionJig(self.canbus, CANBUS_ID_INSERTION)
            self.insertion_servos = InsertionServos(self.canbus, CANBUS_ID_INSERTION_SERVOS)
            self.hose_puller = HosePuller(self.canbus, CANBUS_ID_PULLER)
            self.hose_jig = HoseJig(self.canbus, CANBUS_ID_JIG)
            self.puller_extension = PullerExtension(self.canbus, CANBUS_ID_EXTENSION)
            self.lubrication_feeder = LubricationFeeder(self.canbus, CANBUS_ID_LUBRICATION_FEEDER)
            self.elevator_in = ElevatorIn(self.canbus, CANBUS_ID_ELEVATOR_IN)
            self.pick_and_place = PickAndPlace(self.canbus, CANBUS_ID_PICK_AND_PLACE)
            self.transporter_fuyus = TransporterFuyus(self.canbus, CANBUS_ID_TRANSPORTER_FUYUS)
            
            self.connection_status.set("Conectado")
            self.log_message("Hardware conectado exitosamente")
            
            # Enable test buttons
            for widget in self.root.winfo_children():
                self.enable_test_buttons(widget)
                
        except Exception as e:
            self.log_message(f"Error al conectar hardware: {str(e)}")
            messagebox.showerror("Error de Conexión", f"No se pudo conectar al hardware: {str(e)}")
            
    def enable_test_buttons(self, widget):
        """Recursively enable test buttons"""
        if isinstance(widget, ttk.Button):
            if "Ejecutar" in widget.cget("text") or "Probar" in widget.cget("text") or "Detener" in widget.cget("text") or "Enviar" in widget.cget("text"):
                widget.configure(state=tk.NORMAL)
        
        for child in widget.winfo_children():
            self.enable_test_buttons(child)
            
    def disconnect_hardware(self):
        """Disconnect from hardware"""
        try:
            if self.canbus:
                self.canbus.close_canbus()
                self.canbus = None
            
            self.insertion_jig = None
            self.insertion_servos = None
            self.hose_puller = None
            self.hose_jig = None
            self.puller_extension = None
            self.lubrication_feeder = None
            
            self.connection_status.set("Desconectado")
            self.log_message("Hardware desconectado")
            
            # Disable test buttons
            for widget in self.root.winfo_children():
                self.disable_test_buttons(widget)
                
        except Exception as e:
            self.log_message(f"Error al desconectar: {str(e)}")
            
    def disable_test_buttons(self, widget):
        """Recursively disable test buttons"""
        if isinstance(widget, ttk.Button):
            if "Ejecutar" in widget.cget("text") or "Probar" in widget.cget("text") or "Detener" in widget.cget("text") or "Enviar" in widget.cget("text"):
                widget.configure(state=tk.DISABLED)
        
        for child in widget.winfo_children():
            self.disable_test_buttons(child)
            
    def run_full_cycle(self):
        """Run the full cycle with current parameters"""
        if not self.canbus:
            messagebox.showerror("Error", "Hardware no conectado")
            return
            
        # Update parameters from GUI
        self.save_parameters()
        
        # Run cycle in separate thread to avoid blocking GUI
        thread = threading.Thread(target=self._execute_cycle)
        thread.daemon = True
        thread.start()
        
    def _execute_cycle(self):
        """Execute the cycle with current parameters (runs in separate thread)"""
        try:
            self.log_message("=== INICIANDO CICLO COMPLETO ===")
            
            # Get current parameters
            params = self.current_params.copy()
            
            # Apply offsets
            home_position_z = params['home_position_z'] + params['offset_z']
            home_position_x = params['home_position_x'] + params['offset_x']
            lubrication_position_z = params['lubrication_position_z'] + params['offset_z']
            lubricate_nozzle = params['lubricate_nozzle'] + params['offset_x']
            insertion_position_z = params['insertion_position_z'] + params['offset_z']
            insert_nozzle = params['insert_nozzle'] + params['offset_x']
            librication_position_joint_z = params['librication_position_joint_z'] + params['offset_z']
            lubricate_joint = params['lubricate_joint'] + params['offset_x']
            insertion_position_joint_z = params['insertion_position_joint_z'] + params['offset_z']
            insert_joint = params['insert_joint'] + params['offset_x']
            
            # ===== FEED HOSE =====
            self.log_message("1. Alimentando manguera...")
            result = self.lubrication_feeder.close_hose_holder()
            if result != "success":
                self.log_message(f"ERROR 01 - close_hose_holder: {result}")
                return "error01"
                
            result = self.lubrication_feeder.feed_hose(duration=params['feed_hose_time'])
            if result != "success":
                self.log_message(f"ERROR 02 - feed_hose: {result}")
                return "error02"
                
            time.sleep(1)
            
            result = self.insertion_servos.holder_hose_nozzle_close()
            if result != "success":
                self.log_message(f"ERROR 08 - holder_hose_nozzle_close: {result}")
                return "error08"
                
            result = self.lubrication_feeder.open_hose_holder()
            if result != "success":
                self.log_message(f"ERROR 03 - open_hose_holder: {result}")
                return "error03"
            
            # ===== CLAMPING AND RESET GRIPPER =====
            self.log_message("2. Sujetando y reseteando gripper...")
            result = self.insertion_servos.clamp_nozzle_close()
            if result != "success":
                self.log_message(f"ERROR 08 - clamp_nozzle_close: {result}")
                return "error08"
                
            result = self.insertion_servos.clamp_joint_close()
            if result != "success":
                self.log_message(f"ERROR 19 - clamp_joint_close: {result}")
                return "error19"
                
            result = self.puller_extension.open_gripper()
            if result != "success":
                self.log_message(f"ERROR 20 - open_gripper: {result}")
                return "error20"
            
            # ===== HOMING SLIDER BEFORE MOVEMENT =====
            self.log_message("3. Enviando sliders a home...")
            result = self.insertion_servos.slider_joint_home()
            if result != "success":
                self.log_message(f"ERROR 08 - slider_joint_home: {result}")
                return "error08"
                
            result = self.insertion_servos.slider_nozzle_home()
            if result != "success":
                self.log_message(f"ERROR 08 - slider_nozzle_home: {result}")
                return "error08"
            
            # ===== INSERTION JIG HOME POSITION =====
            self.log_message("4. Moviendo insertion jig a posición home...")
            result = self.hose_jig.insertion_position(False)
            if result != "success":
                self.log_message(f"ERROR 05 - insertion_position: {result}")
                return "error05"
                
            result = self.insertion_jig.move_z_axis(home_position_z)
            if result != "success":
                self.log_message(f"ERROR 01 - move_z_axis home: {result}")
                return "error01"
                
            result = self.insertion_jig.move_x_axis(home_position_x)
            if result != "success":
                self.log_message(f"ERROR 02 - move_x_axis home: {result}")
                return "error02"
            
            # ===== LUBRICATE HOSE =====
            self.log_message("5. Lubricando boquilla...")
            # NOTE: NO repetir holder_hose_nozzle_close() aquí porque ya se hizo en el paso 1
            result = self.insertion_jig.move_z_axis(lubrication_position_z)
            if result != "success":
                self.log_message(f"ERROR 03 - move_z_axis lubrication: {result}")
                return "error03"
                
            result = self.insertion_jig.move_x_axis(lubricate_nozzle)
            if result != "success":
                self.log_message(f"ERROR 04 - move_x_axis lubricate_nozzle: {result}")
                return "error04"
                
            result = self.lubrication_feeder.lubricate_nozzle(params['lubricate_nozzle_time'])
            if result != "success":
                self.log_message(f"ERROR 03.1 - lubricate_nozzle: {result}")
                return "error03.1"
            
            # ===== NOZZLE INSERTION =====
            self.log_message("6. Insertando boquilla...")
            result = self.insertion_jig.move_z_axis(insertion_position_z)
            if result != "success":
                self.log_message(f"ERROR 05 - move_z_axis insertion: {result}")
                return "error05"
                
            result = self.insertion_jig.move_x_axis(insert_nozzle)
            if result != "success":
                self.log_message(f"ERROR 06 - move_x_axis insert_nozzle: {result}")
                return "error06"
                
            result = self.insertion_servos.holder_hose_nozzle_close()
            if result != "success":
                self.log_message(f"ERROR 07 - holder_hose_nozzle_close: {result}")
                return "error07"
                
            result = self.insertion_servos.clamp_nozzle_close()
            if result != "success":
                self.log_message(f"ERROR 08 - clamp_nozzle_close: {result}")
                return "error08"
                
            time.sleep(0.5)
            
            result = self.insertion_servos.slider_nozzle_insertion(self.current_params['slider_nozzle_insertion_position'])
            if result != "success":
                self.log_message(f"ERROR 09 - slider_nozzle_insertion: {result}")
                return "error09"
                
            time.sleep(1)
            
            result = self.insertion_servos.holder_hose_nozzle_open()
            if result != "success":
                self.log_message(f"ERROR 10 - holder_hose_nozzle_open: {result}")
                return "error10"
                
            result = self.insertion_servos.clamp_nozzle_open()
            if result != "success":
                self.log_message(f"ERROR 11 - clamp_nozzle_open: {result}")
                return "error11"
                
            time.sleep(0.5)
            
            result = self.insertion_servos.slider_nozzle_home()
            if result != "success":
                self.log_message(f"ERROR 12 - slider_nozzle_home: {result}")
                return "error12"
            
            # ===== GO TO DOWN POSITION FOR HOSE PULLER =====
            self.log_message("7. Moviendo a posición segura para hose puller...")
            result = self.insertion_jig.move_z_axis(params['insertion_jig_safe_zone'])
            if result != "success":
                self.log_message(f"ERROR 13 - move_z_axis safe_zone: {result}")
                return "error13"
            
            # ===== STARTING PREFEEDER =====
            self.log_message("8. Iniciando prefeeder...")
            result = self.lubrication_feeder.move_pre_feeder(params['preefeder_speed'])
            if result != "success":
                self.log_message(f"ERROR 04 - move_pre_feeder: {result}")
                return "error04"
            
            # ===== PREPARING HOSE PULLER AND HOSE JIG =====
            self.log_message("9. Preparando hose puller y hose jig...")
            result = self.hose_puller.move_z_actuator(params['safe_position'])
            if result != "success":
                self.log_message(f"ERROR 04 - move_z_actuator safe: {result}")
                return "error04"
                
            result = self.hose_puller.move_y_actuator(params['home_y'])
            if result != "success":
                self.log_message(f"ERROR 03 - move_y_actuator home: {result}")
                return "error03"
                
            result = self.hose_jig.insertion_position()
            if result != "success":
                self.log_message(f"ERROR 05 - insertion_position: {result}")
                return "error05"
            
            # ===== PULLING ACTION =====
            self.log_message("10. Ejecutando acción de tirado...")
            result = self.hose_puller.move_y_actuator(params['pickup_y'])
            if result != "success":
                self.log_message(f"ERROR 06 - move_y_actuator pickup: {result}")
                return "error06"
                
            result = self.hose_puller.move_z_actuator(params['z_picking_position'])
            if result != "success":
                self.log_message(f"ERROR 07 - move_z_actuator picking: {result}")
                return "error07"
                
            result = self.puller_extension.close_gripper()
            if result != "success":
                self.log_message(f"ERROR 08 - close_gripper: {result}")
                return "error08"
                
            result = self.hose_puller.move_y_actuator_with_speed(params['before_rise_position'], params['hose_puller_y_speed'])
            if result != "success":
                self.log_message(f"ERROR 09 - move_y_actuator_with_speed before_rise: {result}")
                return "error09"
                
            result = self.hose_puller.move_z_actuator(params['safe_position'])
            if result != "success":
                self.log_message(f"ERROR 10 - move_z_actuator safe: {result}")
                return "error10"
                
            result = self.hose_puller.move_y_actuator_with_speed(params['cutting_position'], params['hose_puller_y_speed'])
            if result != "success":
                self.log_message(f"ERROR 11 - move_y_actuator_with_speed cutting: {result}")
                return "error11"
                
            result = self.hose_puller.move_z_actuator(params['safe_position_over_hose_jig'])
            if result != "success":
                self.log_message(f"ERROR 10 - move_z_actuator safe_over_jig: {result}")
                return "error10"
                
            result = self.hose_puller.move_y_actuator_with_speed(params['wait_y'], params['hose_puller_y_speed'])
            if result != "success":
                self.log_message(f"ERROR 11 - move_y_actuator_with_speed wait: {result}")
                return "error11"
            
            # ===== CUTTING HOSE =====
            self.log_message("11. Cortando manguera...")
            result = self.lubrication_feeder.close_hose_holder()
            if result != "success":
                self.log_message(f"ERROR 01 - close_hose_holder: {result}")
                return "error01"
                
            time.sleep(0.5)
            
            result = self.insertion_servos.activate_cutter()
            if result != "success":
                self.log_message(f"ERROR 12 - activate_cutter: {result}")
                return "error12"
            
            # ===== ALIGNMENT FOR JOINT INSERTION =====
            self.log_message("12. Alineando para inserción de junta...")
            result = self.hose_puller.move_y_actuator_with_speed(params['alignmnet_for_joint'], params['hose_puller_y_speed_for_alignment'])
            if result != "success":
                self.log_message(f"ERROR 13 - move_y_actuator_with_speed alignment: {result}")
                return "error13"
            
            # ===== STOPPING PREFEEDER =====
            self.log_message("13. Deteniendo prefeeder...")
            result = self.lubrication_feeder.move_pre_feeder(0)
            if result != "success":
                self.log_message(f"ERROR 04 - move_pre_feeder stop: {result}")
                return "error04"
            
            # ===== LUBRICATE HOSE ON JOINT AREA =====
            self.log_message("14. Lubricando área de junta...")
            result = self.insertion_servos.holder_hose_joint_close()
            if result != "success":
                self.log_message(f"ERROR 18 - holder_hose_joint_close: {result}")
                return "error18"
                
            result = self.insertion_jig.move_x_axis(lubricate_joint)
            if result != "success":
                self.log_message(f"ERROR 15 - move_x_axis lubricate_joint: {result}")
                return "error15"
                
            result = self.insertion_jig.move_z_axis(librication_position_joint_z)
            if result != "success":
                self.log_message(f"ERROR 14 - move_z_axis lubrication_joint: {result}")
                return "error14"
                
            result = self.lubrication_feeder.lubricate_joint(params['lubricate_joint_time'])
            if result != "success":
                self.log_message(f"ERROR 14.1 - lubricate_joint: {result}")
                return "error14.1"
                
            time.sleep(0.5)
            
            # ===== JOINT INSERTION =====
            self.log_message("15. Insertando junta...")
            result = self.insertion_jig.move_x_axis(insert_joint)
            if result != "success":
                self.log_message(f"ERROR 17 - move_x_axis insert_joint: {result}")
                return "error17"
                
            result = self.insertion_jig.move_z_axis(insertion_position_joint_z)
            if result != "success":
                self.log_message(f"ERROR 16 - move_z_axis insertion_joint: {result}")
                return "error16"
                
            result = self.insertion_servos.clamp_joint_close()
            if result != "success":
                self.log_message(f"ERROR 19 - clamp_joint_close: {result}")
                return "error19"
                
            time.sleep(0.5)
            
            result = self.insertion_servos.slider_joint_insertion(self.current_params['slider_joint_insertion_position'])
            if result != "success":
                self.log_message(f"ERROR 20 - slider_joint_insertion: {result}")
                return "error20"
                
            time.sleep(1)
            
            result = self.insertion_servos.holder_hose_joint_open()
            if result != "success":
                self.log_message(f"ERROR 21 - holder_hose_joint_open: {result}")
                return "error21"
                
            result = self.insertion_servos.clamp_joint_open()
            if result != "success":
                self.log_message(f"ERROR 22 - clamp_joint_open: {result}")
                return "error22"
                
            time.sleep(0.5)
            
            result = self.insertion_servos.slider_joint_home()
            if result != "success":
                self.log_message(f"ERROR 23 - slider_joint_home: {result}")
                return "error23"
            
            # ===== FINISH PULLING ACTION =====
            self.log_message("16. Finalizando acción de tirado...")
            result = self.hose_puller.move_y_actuator(params['home_y'])
            if result != "success":
                self.log_message(f"ERROR 13 - move_y_actuator home_finish: {result}")
                return "error13"
            
            # ===== HOMING FOR FINISH =====
            self.log_message("17. Enviando a home para finalizar...")
            result = self.insertion_jig.move_z_axis(params['insertion_jig_safe_zone'], False)
            if result != "success":
                self.log_message(f"ERROR 24 - move_z_axis safe_zone_finish: {result}")
                return "error24"
                
            result = self.insertion_jig.move_x_axis(0, False)
            if result != "success":
                self.log_message(f"ERROR 25 - move_x_axis home_finish: {result}")
                return "error25"
                
            result = self.hose_puller.move_z_actuator(params['z_home'])
            if result != "success":
                self.log_message(f"ERROR 14 - move_z_actuator z_home: {result}")
                return "error14"
            
            # ===== DELIVERING HOSE =====
            self.log_message("18. Entregando manguera...")
            result = self.hose_jig.gripper_close()
            if result != "success":
                self.log_message(f"ERROR 15 - gripper_close: {result}")
                return "error15"
                
            result = self.puller_extension.open_gripper()
            if result != "success":
                self.log_message(f"ERROR 16 - open_gripper_final: {result}")
                return "error16"
                
            result = self.hose_puller.move_z_actuator(params['safe_position'])
            if result != "success":
                self.log_message(f"ERROR 17 - move_z_actuator safe_final: {result}")
                return "error17"
                
            result = self.hose_jig.deliver_position()
            if result != "success":
                self.log_message(f"ERROR 18 - deliver_position: {result}")
                return "error18"
                
            result = self.hose_puller.move_z_actuator(0)
            if result != "success":
                self.log_message(f"ERROR 20 - move_z_actuator zero: {result}")
                return "error20"
            
            self.log_message("=== CICLO COMPLETADO EXITOSAMENTE ===")
            return "success"
            
        except Exception as e:
            error_msg = f"Error durante la ejecución del ciclo: {str(e)}"
            self.log_message(error_msg)
            return f"error_exception: {str(e)}"
            
    def test_individual_movements(self):
        """Test individual movements"""
        if not self.canbus:
            messagebox.showerror("Error", "Hardware no conectado")
            return
            
        # Create a new window for individual movement tests
        test_window = tk.Toplevel(self.root)
        test_window.title("Pruebas de Movimientos Individuales")
        test_window.geometry("800x600")
        
        # Create notebook for different component tests
        notebook = ttk.Notebook(test_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Insertion Jig Tests
        jig_frame = ttk.Frame(notebook)
        notebook.add(jig_frame, text="Insertion Jig")
        self.create_insertion_jig_tests(jig_frame)
        
        # Hose Puller Tests
        puller_frame = ttk.Frame(notebook)
        notebook.add(puller_frame, text="Hose Puller")
        self.create_hose_puller_tests(puller_frame)
        
        # Insertion Servos Tests
        servos_frame = ttk.Frame(notebook)
        notebook.add(servos_frame, text="Insertion Servos")
        self.create_insertion_servos_tests(servos_frame)
        
        # Lubrication Feeder Tests
        lubrication_frame = ttk.Frame(notebook)
        notebook.add(lubrication_frame, text="Lubrication Feeder")
        self.create_lubrication_feeder_tests(lubrication_frame)
        
        # Hose Jig Tests
        hose_jig_frame = ttk.Frame(notebook)
        notebook.add(hose_jig_frame, text="Hose Jig")
        self.create_hose_jig_tests(hose_jig_frame)
        
        # Puller Extension Tests
        extension_frame = ttk.Frame(notebook)
        notebook.add(extension_frame, text="Puller Extension")
        self.create_puller_extension_tests(extension_frame)
        
    def create_insertion_jig_tests(self, parent):
        """Create insertion jig test controls"""
        ttk.Label(parent, text="Insertion Jig - Movimientos de Ejes", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Z Axis controls
        z_frame = ttk.LabelFrame(parent, text="Eje Z", padding=10)
        z_frame.pack(fill=tk.X, padx=10, pady=5)
        
        z_pos_var = tk.StringVar(value="0")
        ttk.Label(z_frame, text="Posición Z:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(z_frame, textvariable=z_pos_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(z_frame, text="Mover Z", command=lambda: self.test_move_z(z_pos_var.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(z_frame, text="Home Z", command=self.test_home_z).pack(side=tk.LEFT, padx=5)
        
        # X Axis controls
        x_frame = ttk.LabelFrame(parent, text="Eje X", padding=10)
        x_frame.pack(fill=tk.X, padx=10, pady=5)
        
        x_pos_var = tk.StringVar(value="0")
        ttk.Label(x_frame, text="Posición X:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(x_frame, textvariable=x_pos_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(x_frame, text="Mover X", command=lambda: self.test_move_x(x_pos_var.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(x_frame, text="Home X", command=self.test_home_x).pack(side=tk.LEFT, padx=5)
        
        # Quick position buttons
        quick_frame = ttk.LabelFrame(parent, text="Posiciones Rápidas", padding=10)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(quick_frame, text="Posición Home", command=self.test_jig_home_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Posición Lubricación", command=self.test_jig_lubrication_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Posición Inserción", command=self.test_jig_insertion_position).pack(side=tk.LEFT, padx=5)
        
    def create_hose_puller_tests(self, parent):
        """Create hose puller test controls"""
        ttk.Label(parent, text="Hose Puller - Movimientos de Actuadores", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Y Axis controls
        y_frame = ttk.LabelFrame(parent, text="Actuador Y", padding=10)
        y_frame.pack(fill=tk.X, padx=10, pady=5)
        
        y_pos_var = tk.StringVar(value="0")
        y_speed_var = tk.StringVar(value="200")
        ttk.Label(y_frame, text="Posición Y:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(y_frame, textvariable=y_pos_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Label(y_frame, text="Velocidad:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(y_frame, textvariable=y_speed_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(y_frame, text="Mover Y", command=lambda: self.test_move_y(y_pos_var.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(y_frame, text="Mover Y con Velocidad", command=lambda: self.test_move_y_speed(y_pos_var.get(), y_speed_var.get())).pack(side=tk.LEFT, padx=5)
        
        # Hose Sensor Test frame
        sensor_frame = ttk.LabelFrame(parent, text="Sensor de Manguera", padding=10)
        sensor_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Speed input for sensor test
        self.hose_sensor_speed_var = tk.StringVar(value="200")
        ttk.Label(sensor_frame, text="Velocidad:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(sensor_frame, textvariable=self.hose_sensor_speed_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(sensor_frame, text="Mover Y hasta No Detectar Manguera", 
                  command=self.test_move_y_until_no_hose).pack(side=tk.LEFT, padx=5)
        
        # Z Axis controls
        z_frame = ttk.LabelFrame(parent, text="Actuador Z", padding=10)
        z_frame.pack(fill=tk.X, padx=10, pady=5)
        
        z_pos_var = tk.StringVar(value="0")
        ttk.Label(z_frame, text="Posición Z:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(z_frame, textvariable=z_pos_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(z_frame, text="Mover Z", command=lambda: self.test_puller_move_z(z_pos_var.get())).pack(side=tk.LEFT, padx=5)
        
        # Quick position buttons
        quick_frame = ttk.LabelFrame(parent, text="Posiciones Rápidas", padding=10)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(quick_frame, text="Home Y", command=self.test_puller_home_y).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Pickup Position", command=self.test_puller_pickup).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Cutting Position", command=self.test_puller_cutting).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Safe Position", command=self.test_puller_safe).pack(side=tk.LEFT, padx=5)
        
    def create_insertion_servos_tests(self, parent):
        """Create insertion servos test controls"""
        ttk.Label(parent, text="Insertion Servos - Control de Servos", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Nozzle controls
        nozzle_frame = ttk.LabelFrame(parent, text="Controles de Boquilla", padding=10)
        nozzle_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(nozzle_frame, text="Holder Nozzle Close", command=self.test_holder_nozzle_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(nozzle_frame, text="Holder Nozzle Open", command=self.test_holder_nozzle_open).pack(side=tk.LEFT, padx=5)
        ttk.Button(nozzle_frame, text="Clamp Nozzle Close", command=self.test_clamp_nozzle_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(nozzle_frame, text="Clamp Nozzle Open", command=self.test_clamp_nozzle_open).pack(side=tk.LEFT, padx=5)
        
        # Joint controls
        joint_frame = ttk.LabelFrame(parent, text="Controles de Junta", padding=10)
        joint_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(joint_frame, text="Holder Joint Close", command=self.test_holder_joint_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(joint_frame, text="Holder Joint Open", command=self.test_holder_joint_open).pack(side=tk.LEFT, padx=5)
        ttk.Button(joint_frame, text="Clamp Joint Close", command=self.test_clamp_joint_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(joint_frame, text="Clamp Joint Open", command=self.test_clamp_joint_open).pack(side=tk.LEFT, padx=5)
        
        # Slider controls
        slider_frame = ttk.LabelFrame(parent, text="Controles de Slider", padding=10)
        slider_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(slider_frame, text="Slider Nozzle Home", command=self.test_slider_nozzle_home).pack(side=tk.LEFT, padx=5)
        ttk.Button(slider_frame, text="Slider Nozzle Insertion", command=self.test_slider_nozzle_insertion).pack(side=tk.LEFT, padx=5)
        ttk.Button(slider_frame, text="Slider Nozzle Component Placement", command=self.test_slider_nozzle_component_placement).pack(side=tk.LEFT, padx=5)
        ttk.Button(slider_frame, text="Slider Joint Home", command=self.test_slider_joint_home).pack(side=tk.LEFT, padx=5)
        ttk.Button(slider_frame, text="Slider Joint Insertion", command=self.test_slider_joint_insertion).pack(side=tk.LEFT, padx=5)
        ttk.Button(slider_frame, text="Slider Joint Component Placement", command=self.test_slider_joint_component_placement).pack(side=tk.LEFT, padx=5)
        
        # Cutter control
        cutter_frame = ttk.LabelFrame(parent, text="Control de Cortador", padding=10)
        cutter_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(cutter_frame, text="Activar Cortador", command=self.test_activate_cutter).pack(side=tk.LEFT, padx=5)
        
    def create_lubrication_feeder_tests(self, parent):
        """Create lubrication feeder test controls"""
        ttk.Label(parent, text="Lubrication Feeder - Control de Alimentación y Lubricación", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Hose holder controls
        holder_frame = ttk.LabelFrame(parent, text="Sujetador de Manguera", padding=10)
        holder_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(holder_frame, text="Cerrar Sujetador", command=self.test_close_hose_holder).pack(side=tk.LEFT, padx=5)
        ttk.Button(holder_frame, text="Abrir Sujetador", command=self.test_open_hose_holder).pack(side=tk.LEFT, padx=5)
        
        # Feed controls
        feed_frame = ttk.LabelFrame(parent, text="Alimentación de Manguera", padding=10)
        feed_frame.pack(fill=tk.X, padx=10, pady=5)
        
        feed_time_var = tk.StringVar(value="3.15")
        ttk.Label(feed_frame, text="Tiempo (s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(feed_frame, textvariable=feed_time_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(feed_frame, text="Alimentar Manguera", command=lambda: self.test_feed_hose(feed_time_var.get())).pack(side=tk.LEFT, padx=5)
        
        # Prefeeder controls
        prefeeder_frame = ttk.LabelFrame(parent, text="Pre-alimentador", padding=10)
        prefeeder_frame.pack(fill=tk.X, padx=10, pady=5)
        
        prefeeder_speed_var = tk.StringVar(value="50")
        ttk.Label(prefeeder_frame, text="Velocidad:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(prefeeder_frame, textvariable=prefeeder_speed_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(prefeeder_frame, text="Mover Pre-alimentador", command=lambda: self.test_move_prefeeder(prefeeder_speed_var.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(prefeeder_frame, text="Detener Pre-alimentador", command=self.test_stop_prefeeder).pack(side=tk.LEFT, padx=5)
        
        # Lubrication controls
        lubrication_frame = ttk.LabelFrame(parent, text="Lubricación", padding=10)
        lubrication_frame.pack(fill=tk.X, padx=10, pady=5)
        
        nozzle_time_var = tk.StringVar(value="5")
        joint_time_var = tk.StringVar(value="5")
        ttk.Label(lubrication_frame, text="Tiempo Boquilla (s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(lubrication_frame, textvariable=nozzle_time_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(lubrication_frame, text="Lubricar Boquilla", command=lambda: self.test_lubricate_nozzle(nozzle_time_var.get())).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(lubrication_frame, text="Tiempo Junta (s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(lubrication_frame, textvariable=joint_time_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(lubrication_frame, text="Lubricar Junta", command=lambda: self.test_lubricate_joint(joint_time_var.get())).pack(side=tk.LEFT, padx=5)
        
    def create_hose_jig_tests(self, parent):
        """Create hose jig test controls"""
        ttk.Label(parent, text="Hose Jig - Control de Posiciones", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Position controls
        position_frame = ttk.LabelFrame(parent, text="Posiciones", padding=10)
        position_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(position_frame, text="Posición Inserción", command=self.test_hose_jig_insertion).pack(side=tk.LEFT, padx=5)
        ttk.Button(position_frame, text="Posición Entrega", command=self.test_hose_jig_deliver).pack(side=tk.LEFT, padx=5)
        
        # Gripper controls
        gripper_frame = ttk.LabelFrame(parent, text="Gripper", padding=10)
        gripper_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(gripper_frame, text="Cerrar Gripper", command=self.test_hose_jig_gripper_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(gripper_frame, text="Abrir Gripper", command=self.test_hose_jig_gripper_open).pack(side=tk.LEFT, padx=5)
        
    def create_puller_extension_tests(self, parent):
        """Create puller extension test controls"""
        ttk.Label(parent, text="Puller Extension - Control de Gripper", font=("Arial", 12, "bold")).pack(pady=10)
        
        # Gripper controls
        gripper_frame = ttk.LabelFrame(parent, text="Gripper Extension", padding=10)
        gripper_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(gripper_frame, text="Cerrar Gripper", command=self.test_extension_gripper_close).pack(side=tk.LEFT, padx=5)
        ttk.Button(gripper_frame, text="Abrir Gripper", command=self.test_extension_gripper_open).pack(side=tk.LEFT, padx=5)
        
    # Test methods for Insertion Jig
    def test_move_z(self, position):
        try:
            pos = int(position)
            result = self.insertion_jig.move_z_axis(pos)
            self.log_message(f"Mover Z a {pos}: {result}")
        except ValueError:
            self.log_message("Error: Posición Z inválida")
            
    def test_move_x(self, position):
        try:
            pos = int(position)
            result = self.insertion_jig.move_x_axis(pos)
            self.log_message(f"Mover X a {pos}: {result}")
        except ValueError:
            self.log_message("Error: Posición X inválida")
            
    def test_home_z(self):
        result = self.insertion_jig.move_z_axis(self.current_params['home_position_z'] + self.current_params['offset_z'])
        self.log_message(f"Home Z: {result}")
        
    def test_home_x(self):
        result = self.insertion_jig.move_x_axis(self.current_params['home_position_x'] + self.current_params['offset_x'])
        self.log_message(f"Home X: {result}")
        
    def test_jig_home_position(self):
        self.test_home_z()
        self.test_home_x()
        
    def test_jig_lubrication_position(self):
        result1 = self.insertion_jig.move_z_axis(self.current_params['lubrication_position_z'] + self.current_params['offset_z'])
        result2 = self.insertion_jig.move_x_axis(self.current_params['lubricate_nozzle'] + self.current_params['offset_x'])
        self.log_message(f"Posición Lubricación Z: {result1}, X: {result2}")
        
    def test_jig_insertion_position(self):
        result1 = self.insertion_jig.move_z_axis(self.current_params['insertion_position_z'] + self.current_params['offset_z'])
        result2 = self.insertion_jig.move_x_axis(self.current_params['insert_nozzle'] + self.current_params['offset_x'])
        self.log_message(f"Posición Inserción Z: {result1}, X: {result2}")
        
    # Test methods for Hose Puller
    def test_move_y(self, position):
        try:
            pos = int(position)
            result = self.hose_puller.move_y_actuator(pos)
            self.log_message(f"Mover Y a {pos}: {result}")
        except ValueError:
            self.log_message("Error: Posición Y inválida")
            
    def test_move_y_speed(self, position, speed):
        try:
            pos = int(position)
            spd = int(speed)
            result = self.hose_puller.move_y_actuator_with_speed(pos, spd)
            self.log_message(f"Mover Y a {pos} con velocidad {spd}: {result}")
        except ValueError:
            self.log_message("Error: Posición Y o velocidad inválida")
    
    def test_move_y_until_no_hose(self):
        """Test method for moving Y axis until no hose is detected"""
        try:
            speed = int(self.hose_sensor_speed_var.get())
            direction = 0  # Default direction (forward)
            acceleration = 236  # Default acceleration
            
            self.log_message(f"Iniciando movimiento Y hasta no detectar manguera - Velocidad: {speed}")
            result = self.hose_puller.move_y_axis_until_no_hose(speed, direction, acceleration)
            
            if result:
                status, hose_detected = result
                status_msg = "Éxito" if status == 1 else "Error"
                hose_msg = "Detectada" if hose_detected == 1 else "No detectada"
                self.log_message(f"Resultado: Estado={status_msg}, Manguera={hose_msg}")
            else:
                self.log_message("Error: No se recibió respuesta del comando")
                
        except ValueError:
            self.log_message("Error: Velocidad debe ser un número entero")
        except Exception as e:
            self.log_message(f"Error al ejecutar movimiento Y hasta no detectar manguera: {e}")
            
    def test_puller_move_z(self, position):
        try:
            pos = int(position)
            result = self.hose_puller.move_z_actuator(pos)
            self.log_message(f"Mover Z Puller a {pos}: {result}")
        except ValueError:
            self.log_message("Error: Posición Z inválida")
            
    def test_puller_home_y(self):
        result = self.hose_puller.move_y_actuator(self.current_params['home_y'])
        self.log_message(f"Home Y Puller: {result}")
        
    def test_puller_pickup(self):
        result = self.hose_puller.move_y_actuator(self.current_params['pickup_y'])
        self.log_message(f"Pickup Position: {result}")
        
    def test_puller_cutting(self):
        result = self.hose_puller.move_y_actuator(self.current_params['cutting_position'])
        self.log_message(f"Cutting Position: {result}")
        
    def test_puller_safe(self):
        result = self.hose_puller.move_z_actuator(self.current_params['safe_position'])
        self.log_message(f"Safe Position: {result}")
        
    # Test methods for Insertion Servos
    def test_holder_nozzle_close(self):
        result = self.insertion_servos.holder_hose_nozzle_close()
        self.log_message(f"Holder Nozzle Close: {result}")
        
    def test_holder_nozzle_open(self):
        result = self.insertion_servos.holder_hose_nozzle_open()
        self.log_message(f"Holder Nozzle Open: {result}")
        
    def test_clamp_nozzle_close(self):
        result = self.insertion_servos.clamp_nozzle_close()
        self.log_message(f"Clamp Nozzle Close: {result}")
        
    def test_clamp_nozzle_open(self):
        result = self.insertion_servos.clamp_nozzle_open()
        self.log_message(f"Clamp Nozzle Open: {result}")
        
    def test_holder_joint_close(self):
        result = self.insertion_servos.holder_hose_joint_close()
        self.log_message(f"Holder Joint Close: {result}")
        
    def test_holder_joint_open(self):
        result = self.insertion_servos.holder_hose_joint_open()
        self.log_message(f"Holder Joint Open: {result}")
        
    def test_clamp_joint_close(self):
        result = self.insertion_servos.clamp_joint_close()
        self.log_message(f"Clamp Joint Close: {result}")
        
    def test_clamp_joint_open(self):
        result = self.insertion_servos.clamp_joint_open()
        self.log_message(f"Clamp Joint Open: {result}")
        
    def test_slider_nozzle_home(self):
        result = self.insertion_servos.slider_nozzle_home()
        self.log_message(f"Slider Nozzle Home: {result}")
        
    def test_slider_nozzle_insertion(self):
        result = self.insertion_servos.slider_nozzle_insertion(self.current_params['slider_nozzle_insertion_position'])
        self.log_message(f"Slider Nozzle Insertion (position: {self.current_params['slider_nozzle_insertion_position']}): {result}")
        
    def test_slider_nozzle_component_placement(self):
        result = self.insertion_servos.slider_nozzle_insertion(self.current_params['slider_nozzle_component_placement_position'])
        self.log_message(f"Slider Nozzle Component Placement (position: {self.current_params['slider_nozzle_component_placement_position']}): {result}")
        
    def test_slider_joint_home(self):
        result = self.insertion_servos.slider_joint_home()
        self.log_message(f"Slider Joint Home: {result}")
        
    def test_slider_joint_insertion(self):
        result = self.insertion_servos.slider_joint_insertion(self.current_params['slider_joint_insertion_position'])
        self.log_message(f"Slider Joint Insertion (position: {self.current_params['slider_joint_insertion_position']}): {result}")
        
    def test_slider_joint_component_placement(self):
        result = self.insertion_servos.slider_joint_insertion(self.current_params['slider_joint_component_placement_position'])
        self.log_message(f"Slider Joint Component Placement (position: {self.current_params['slider_joint_component_placement_position']}): {result}")
        
    def test_activate_cutter(self):
        result = self.insertion_servos.activate_cutter()
        self.log_message(f"Activate Cutter: {result}")
        
    # Test methods for Lubrication Feeder
    def test_close_hose_holder(self):
        result = self.lubrication_feeder.close_hose_holder()
        self.log_message(f"Close Hose Holder: {result}")
        
    def test_open_hose_holder(self):
        result = self.lubrication_feeder.open_hose_holder()
        self.log_message(f"Open Hose Holder: {result}")
        
    def test_feed_hose(self, duration):
        try:
            dur = float(duration)
            result = self.lubrication_feeder.feed_hose(duration=dur)
            self.log_message(f"Feed Hose ({dur}s): {result}")
        except ValueError:
            self.log_message("Error: Duración inválida")
            
    def test_move_prefeeder(self, speed):
        try:
            spd = int(speed)
            result = self.lubrication_feeder.move_pre_feeder(spd)
            self.log_message(f"Move Prefeeder ({spd}): {result}")
        except ValueError:
            self.log_message("Error: Velocidad inválida")
            
    def test_stop_prefeeder(self):
        result = self.lubrication_feeder.move_pre_feeder(0)
        self.log_message(f"Stop Prefeeder: {result}")
        
    def test_lubricate_nozzle(self, duration):
        try:
            dur = float(duration)
            result = self.lubrication_feeder.lubricate_nozzle(dur)
            self.log_message(f"Lubricate Nozzle ({dur}s): {result}")
        except ValueError:
            self.log_message("Error: Duración inválida")
            
    def test_lubricate_joint(self, duration):
        try:
            dur = float(duration)
            result = self.lubrication_feeder.lubricate_joint(dur)
            self.log_message(f"Lubricate Joint ({dur}s): {result}")
        except ValueError:
            self.log_message("Error: Duración inválida")
            
    # Test methods for Hose Jig
    def test_hose_jig_insertion(self):
        result = self.hose_jig.insertion_position()
        self.log_message(f"Hose Jig Insertion Position: {result}")
        
    def test_hose_jig_deliver(self):
        result = self.hose_jig.deliver_position()
        self.log_message(f"Hose Jig Deliver Position: {result}")
        
    def test_hose_jig_gripper_close(self):
        result = self.hose_jig.gripper_close()
        self.log_message(f"Hose Jig Gripper Close: {result}")
        
    def test_hose_jig_gripper_open(self):
        result = self.hose_jig.gripper_open()
        self.log_message(f"Hose Jig Gripper Open: {result}")
        
    # Test methods for Puller Extension
    def test_extension_gripper_close(self):
        result = self.puller_extension.close_gripper()
        self.log_message(f"Extension Gripper Close: {result}")
        
    def test_extension_gripper_open(self):
        result = self.puller_extension.open_gripper()
        self.log_message(f"Extension Gripper Open: {result}")
        
    def stop_test(self):
        """Stop current test"""
        self.log_message("=== PRUEBA DETENIDA POR USUARIO ===")
    
    def send_all_to_home(self):
        """Send all hardware components to their home positions"""
        if not self.canbus:
            messagebox.showerror("Error", "Hardware no conectado")
            return
        
        try:
            self.log_message("=== INICIANDO SECUENCIA DE ENVÍO A POSICIONES HOME ===")
            
            # Send insertion servos to home first
            if self.insertion_servos:
                self.log_message("Enviando servos de inserción a home...")
                result = self.insertion_servos.slider_joint_home()
                self.log_message(f"Slider joint home: {result}")
                
                result = self.insertion_servos.slider_nozzle_home()
                self.log_message(f"Slider nozzle home: {result}")
            
            # Send ElevatorIn to home positions
            if self.elevator_in:
                self.log_message("Enviando Elevator In a home...")
                result = self.elevator_in.home_gantry_z()
                self.log_message(f"Elevator In Gantry Z home: {result}")
                
                result = self.elevator_in.home_gantry_x()
                self.log_message(f"Elevator In Gantry X home: {result}")
                
                result = self.elevator_in.home_gantry_y()
                self.log_message(f"Elevator In Gantry Y home: {result}")
                
                result = self.elevator_in.home_elevator_z()
                self.log_message(f"Elevator In Elevator Z home: {result}")
            
            # Send PickAndPlace to home positions
            if self.pick_and_place:
                self.log_message("Enviando Pick and Place a home...")
                result = self.pick_and_place.home_z_axis()
                self.log_message(f"Pick and Place Z home: {result}")
                
                result = self.pick_and_place.home_x_axis()
                self.log_message(f"Pick and Place X home: {result}")
            
            # Send HoseJig to home
            if self.hose_jig:
                self.log_message("Enviando Hose Jig a home...")
                result = self.hose_jig.go_home()
                self.log_message(f"Hose Jig home: {result}")
            
            # Send HosePuller to home positions
            if self.hose_puller:
                self.log_message("Enviando Hose Puller a home...")
                result = self.hose_puller.home_y_axis()
                self.log_message(f"Hose Puller Y home: {result}")
                
                result = self.hose_puller.home_z_axis()
                self.log_message(f"Hose Puller Z home: {result}")
            
            # Send InsertionJig to home positions
            if self.insertion_jig:
                self.log_message("Enviando Insertion Jig a home...")
                result = self.insertion_jig.home_x_axis_go_home()
                self.log_message(f"Insertion Jig X home: {result}")
                
                result = self.insertion_jig.home_z_axis_go_home()
                self.log_message(f"Insertion Jig Z home: {result}")
            
            # # Send TransporterFuyus to home position
            # if self.transporter_fuyus:
            #     self.log_message("Enviando Transporter Fuyus a home...")
            #     result = self.transporter_fuyus.home_x_axis()
            #     self.log_message(f"Transporter Fuyus X home: {result}")
            
            self.log_message("=== SECUENCIA DE ENVÍO A HOME COMPLETADA EXITOSAMENTE ===")
            
        except Exception as e:
            error_msg = f"Error durante la secuencia de home: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("Error", error_msg)

def main():
    root = tk.Tk()
    app = CycleParameterTester(root)
    root.mainloop()

if __name__ == "__main__":
    main()