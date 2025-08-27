import tkinter as tk
from tkinter import ttk, messagebox
import threading
import json
import os
from automation_functions import oneCycle, testHome, initialize_devices

class AutomationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PH2K Automation Control")
        self.root.geometry("900x800")
        
        # Initialize device instances
        self.devices = {
            'insertion_jig': None,
            'insertion_servos': None,
            'hose_puller': None,
            'hose_jig': None,
            'puller_extension': None,
            'pick_and_place': None,
            'lubrication_feeder': None,
            'elevator_in': None
        }
        
        # Parameters dictionary to store custom values
        self.custom_params = {}
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Create tabs
        self.setup_control_tab()
        self.setup_parameters_tab()
        
        # Load saved parameters
        self.load_parameters()
        
    def setup_control_tab(self):
        """Setup the main control tab"""
        self.control_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.control_frame, text="Control Panel")
        
        # Main frame with padding
        main_frame = ttk.Frame(self.control_frame, padding="20")
        main_frame.pack(fill="both", expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="PH2K Automation Control", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Initialize Devices Button
        init_button = ttk.Button(main_frame, text="Initialize Devices", 
                                command=self.initialize_devices_handler)
        init_button.grid(row=1, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # One Cycle Button
        cycle_button = ttk.Button(main_frame, text="Run One Cycle", 
                                 command=self.run_one_cycle)
        cycle_button.grid(row=2, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Test Home Button
        home_button = ttk.Button(main_frame, text="Test Home Functions", 
                                command=self.run_test_home)
        home_button.grid(row=3, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Status Text Area
        status_label = ttk.Label(main_frame, text="Status:")
        status_label.grid(row=4, column=0, sticky=tk.W, pady=(20, 5))
        
        self.status_text = tk.Text(main_frame, height=15, width=70)
        self.status_text.grid(row=5, column=0, columnspan=2, pady=5)
        
        # Scrollbar for status text
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=self.status_text.yview)
        scrollbar.grid(row=5, column=2, sticky=(tk.N, tk.S))
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        # Configure grid weights
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)
        
    def setup_parameters_tab(self):
        """Setup the parameters configuration tab"""
        self.params_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.params_frame, text="Parameters")
        
        # Create scrollable frame
        canvas = tk.Canvas(self.params_frame)
        scrollbar_params = ttk.Scrollbar(self.params_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar_params.set)
        
        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar_params.pack(side="right", fill="y")
        
        # Parameters form
        form_frame = ttk.Frame(scrollable_frame, padding="20")
        form_frame.pack(fill="both", expand=True)
        
        # Title
        title_label = ttk.Label(form_frame, text="OneCycle Parameters", 
                               font=('Arial', 14, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Parameter definitions with default values
        self.param_vars = {}
        self.param_entries = {}
        
        parameters = [
            # Position parameters
            ('home_position_x', 'Home Position X', 0),
            ('home_position_z', 'Home Position Z', 0),
            ('lubrication_position_z', 'Lubrication Position Z', 2000),
            ('lubricate_nozzle', 'Lubricate Nozzle Position', 1500),
            ('insertion_position_z', 'Insertion Position Z', 1000),
            ('insert_nozzle', 'Insert Nozzle Position', 2500),
            ('lubricate_joint', 'Lubricate Joint Position', 3500),
            ('librication_position_joint_z', 'Lubrication Position Joint Z', 2000),
            ('insert_joint', 'Insert Joint Position', 4500),
            ('insertion_position_joint_z', 'Insertion Position Joint Z', 1000),
            
            # Hose puller parameters
            ('safe_position', 'Safe Position', 2000),
            ('home_y', 'Home Y Position', 0),
            ('pickup_y', 'Pickup Y Position', 3000),
            ('wait_y', 'Wait Y Position', 1500),
            ('z_home', 'Z Home Position', 0),
            
            # Timing and movement parameters
            ('feeding_time', 'Feeding Time (s)', 1.53),
            ('lubrication_time_nozzle', 'Lubrication Time Nozzle (s)', 5),
            ('lubrication_time_joint', 'Lubrication Time Joint (s)', 6),
            ('insertion_safe_position', 'Insertion Safe Position', 4000),
            ('prefeeder_speed', 'Prefeeder Speed', 50),
            ('pick_up_gap', 'Pick Up Gap', 30),
            ('after_pickup', 'After Pickup Distance', 500),
            ('position_to_second_rise', 'Position to Second Rise', 1900),
            ('second_rise', 'Second Rise Distance', 25),
            ('joint_pull', 'Joint Pull Distance', 687)
        ]
        
        # Create input fields
        row = 1
        for param_key, param_label, default_value in parameters:
            # Label
            label = ttk.Label(form_frame, text=param_label + ":")
            label.grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=2)
            
            # Entry variable
            var = tk.StringVar(value=str(default_value))
            self.param_vars[param_key] = var
            
            # Entry widget
            entry = ttk.Entry(form_frame, textvariable=var, width=15)
            entry.grid(row=row, column=1, padx=(0, 10), pady=2)
            self.param_entries[param_key] = entry
            
            # Unit label (if applicable)
            unit = ""
            if "time" in param_key.lower():
                unit = "seconds"
            elif "speed" in param_key.lower():
                unit = "units/s"
            else:
                unit = "units"
            
            unit_label = ttk.Label(form_frame, text=unit, foreground="gray")
            unit_label.grid(row=row, column=2, sticky=tk.W, pady=2)
            
            row += 1
        
        # Buttons frame
        buttons_frame = ttk.Frame(form_frame)
        buttons_frame.grid(row=row, column=0, columnspan=3, pady=20)
        
        # Save Parameters Button
        save_button = ttk.Button(buttons_frame, text="Save Parameters", 
                                command=self.save_parameters)
        save_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Load Parameters Button
        load_button = ttk.Button(buttons_frame, text="Load Parameters", 
                                command=self.load_parameters)
        load_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Reset to Defaults Button
        reset_button = ttk.Button(buttons_frame, text="Reset to Defaults", 
                                 command=self.reset_parameters)
        reset_button.pack(side=tk.LEFT)
        
    def get_custom_parameters(self):
        """Get custom parameters from the form"""
        custom_params = {}
        for param_key, var in self.param_vars.items():
            try:
                value = float(var.get())
                custom_params[param_key] = value
            except ValueError:
                messagebox.showerror("Error", f"Invalid value for {param_key}: {var.get()}")
                return None
        return custom_params
        
    def save_parameters(self):
        """Save parameters to JSON file"""
        params = self.get_custom_parameters()
        if params is None:
            return
            
        try:
            with open('automation_parameters.json', 'w') as f:
                json.dump(params, f, indent=2)
            messagebox.showinfo("Success", "Parameters saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save parameters: {str(e)}")
            
    def load_parameters(self):
        """Load parameters from JSON file"""
        try:
            if os.path.exists('automation_parameters.json'):
                with open('automation_parameters.json', 'r') as f:
                    params = json.load(f)
                
                for param_key, value in params.items():
                    if param_key in self.param_vars:
                        self.param_vars[param_key].set(str(value))
                        
                self.log_status("Parameters loaded from file.")
        except Exception as e:
            self.log_status(f"Failed to load parameters: {str(e)}")
            
    def reset_parameters(self):
        """Reset all parameters to default values"""
        defaults = {
            'home_position_x': 0, 'home_position_z': 0, 'lubrication_position_z': 2000,
            'lubricate_nozzle': 1500, 'insertion_position_z': 1000, 'insert_nozzle': 2500,
            'lubricate_joint': 3500, 'librication_position_joint_z': 2000, 'insert_joint': 4500,
            'insertion_position_joint_z': 1000, 'safe_position': 2000, 'home_y': 0,
            'pickup_y': 3000, 'wait_y': 1500, 'z_home': 0, 'feeding_time': 1.53,
            'lubrication_time_nozzle': 5, 'lubrication_time_joint': 6, 'insertion_safe_position': 4000,
            'prefeeder_speed': 50, 'pick_up_gap': 30, 'after_pickup': 500,
            'position_to_second_rise': 1900, 'second_rise': 25, 'joint_pull': 687
        }
        
        for param_key, default_value in defaults.items():
            if param_key in self.param_vars:
                self.param_vars[param_key].set(str(default_value))
                
        messagebox.showinfo("Reset", "Parameters reset to default values.")
        
    def log_status(self, message):
        """Add a message to the status text area"""
        if hasattr(self, 'status_text'):
            self.status_text.insert(tk.END, f"{message}\n")
            self.status_text.see(tk.END)
            self.root.update_idletasks()
        
    def initialize_devices_handler(self):
        """Initialize device instances"""
        self.log_status("Initializing devices...")
        self.log_status("WARNING: Device initialization not implemented yet.")
        self.log_status("You need to replace the device initialization code with actual device instances.")
        
        # Initialize the automation functions with device instances
        initialize_devices(self.devices)
        
    def run_one_cycle(self):
        """Run one complete automation cycle in a separate thread"""
        if not any(self.devices.values()):
            messagebox.showwarning("Warning", "Please initialize devices first!")
            return
            
        # Get custom parameters
        custom_params = self.get_custom_parameters()
        if custom_params is None:
            return
            
        def cycle_thread():
            try:
                self.log_status("Starting one cycle with custom parameters...")
                result = oneCycle(custom_params)
                self.log_status(f"One cycle completed with result: {result}")
            except Exception as e:
                self.log_status(f"Error during one cycle: {str(e)}")
                
        # Run in separate thread to prevent GUI freezing
        thread = threading.Thread(target=cycle_thread)
        thread.daemon = True
        thread.start()
        
    def run_test_home(self):
        """Run test home functions in a separate thread"""
        if not any(self.devices.values()):
            messagebox.showwarning("Warning", "Please initialize devices first!")
            return
            
        def home_thread():
            try:
                self.log_status("Starting test home functions...")
                result = testHome()
                self.log_status(f"Test home completed with result: {result}")
            except Exception as e:
                self.log_status(f"Error during test home: {str(e)}")
                
        # Run in separate thread to prevent GUI freezing
        thread = threading.Thread(target=home_thread)
        thread.daemon = True
        thread.start()

def main():
    root = tk.Tk()
    app = AutomationGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()