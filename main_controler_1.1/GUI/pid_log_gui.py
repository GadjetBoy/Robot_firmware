import serial
import threading
import re
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import matplotlib
matplotlib.use('TkAgg')

class PIDPlotter:
    def __init__(self, serial_port='COM13', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = False
        
        # Data storage for 8 motors
        self.num_motors = 8
        self.time_data = [[] for _ in range(self.num_motors)]
        self.p_data = [[] for _ in range(self.num_motors)]
        self.i_data = [[] for _ in range(self.num_motors)]
        self.d_data = [[] for _ in range(self.num_motors)]
        self.out_data = [[] for _ in range(self.num_motors)]
        self.max_points = 500  # Keep last 500 points
        
        # Current values display for all motors
        self.current_values = [{
            'p': 0.0, 'i': 0.0, 'd': 0.0, 'out': 0.0, 
            'error': 0.0, 'position': 0, 'target': 0
        } for _ in range(self.num_motors)]
        
        # GUI setup
        self.root = tk.Tk()
        self.root.title("ESP32 PID Monitor - 8 Motors")
        self.root.geometry("1400x900")
        
        self.setup_gui()
        self.setup_serial()
        
    def setup_gui(self):
        # Create main frames
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Top frame for controls and current values
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Left: Motor selector and controls
        control_frame = ttk.LabelFrame(top_frame, text="Motor Controls", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Motor selector
        ttk.Label(control_frame, text="Monitor Motor:", font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        self.motor_var = tk.StringVar(value="0")
        motor_combo = ttk.Combobox(control_frame, textvariable=self.motor_var, 
                                 values=[str(i) for i in range(self.num_motors)], 
                                 width=10, state="readonly")
        motor_combo.pack(pady=5)
        motor_combo.bind('<<ComboboxSelected>>', self.on_motor_changed)
        
        # Control buttons
        ttk.Button(control_frame, text="Clear All Plots", command=self.clear_all_plots).pack(pady=2, fill=tk.X)
        ttk.Button(control_frame, text="Clear Current Plot", command=self.clear_current_plot).pack(pady=2, fill=tk.X)
        ttk.Button(control_frame, text="Reconnect Serial", command=self.setup_serial).pack(pady=2, fill=tk.X)
        
        # Right: Current values display
        values_frame = ttk.LabelFrame(top_frame, text="Current PID Values", padding=10)
        values_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create a grid for values display
        self.setup_values_grid(values_frame)
        
        # Plot frame
        plot_frame = ttk.LabelFrame(main_frame, text="PID Values Over Time")
        plot_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.setup_plot()
        
        # Embed plot in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Status frame
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.status_label = ttk.Label(status_frame, text="Disconnected")
        self.status_label.pack(side=tk.LEFT)
        
        # Add motor activity indicator
        self.activity_vars = [tk.StringVar(value="●") for _ in range(self.num_motors)]
        activity_frame = ttk.Frame(status_frame)
        activity_frame.pack(side=tk.RIGHT)
        ttk.Label(activity_frame, text="Motor Activity:").pack(side=tk.LEFT, padx=(10, 5))
        for i in range(self.num_motors):
            lbl = ttk.Label(activity_frame, textvariable=self.activity_vars[i], 
                           foreground='red', font=('Arial', 12))
            lbl.pack(side=tk.LEFT, padx=2)
        
    def setup_values_grid(self, parent):
        # Create a grid to display all motor values
        grid_frame = ttk.Frame(parent)
        grid_frame.pack(fill=tk.BOTH, expand=True)
        
        # Headers
        headers = ["Motor", "Error", "P", "I", "D", "Output", "Position", "Target"]
        for col, header in enumerate(headers):
            ttk.Label(grid_frame, text=header, font=('Arial', 9, 'bold')).grid(
                row=0, column=col, padx=5, pady=2, sticky=tk.W)
        
        # Create value labels for each motor
        self.value_labels = []
        for motor in range(self.num_motors):
            row_labels = []
            for col in range(len(headers)):
                lbl = ttk.Label(grid_frame, text="0.00", font=('Arial', 9))
                lbl.grid(row=motor+1, column=col, padx=5, pady=1, sticky=tk.W)
                row_labels.append(lbl)
            self.value_labels.append(row_labels)
            
        # Highlight the currently selected motor row
        self.highlight_current_motor()
        
    def highlight_current_motor(self):
        for motor in range(self.num_motors):
            for label in self.value_labels[motor]:
                if str(motor) == self.motor_var.get():
                    label.configure(background='light blue')
                else:
                    label.configure(background='')
        
    def setup_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('Time (samples)')
        self.ax.set_ylabel('PID Values')
        self.ax.set_title(f'Real-time PID Components - Motor {self.motor_var.get()}')
        self.ax.grid(True, alpha=0.3)
        
        # Create empty plots
        self.p_line, = self.ax.plot([], [], 'b-', label='Proportional (P)', linewidth=2)
        self.i_line, = self.ax.plot([], [], 'g-', label='Integral (I)', linewidth=2) 
        self.d_line, = self.ax.plot([], [], 'r-', label='Derivative (D)', linewidth=2)
        self.out_line, = self.ax.plot([], [], 'purple', label='Output', linewidth=2, linestyle='--')
        
        self.ax.legend(loc='upper right')
        
    def on_motor_changed(self, event=None):
        self.highlight_current_motor()
        self.setup_plot()
        self.update_plot()
        
    def clear_current_plot(self):
        motor = int(self.motor_var.get())
        self.time_data[motor].clear()
        self.p_data[motor].clear()
        self.i_data[motor].clear()
        self.d_data[motor].clear()
        self.out_data[motor].clear()
        self.update_plot()
        
    def clear_all_plots(self):
        for motor in range(self.num_motors):
            self.time_data[motor].clear()
            self.p_data[motor].clear()
            self.i_data[motor].clear()
            self.d_data[motor].clear()
            self.out_data[motor].clear()
        self.update_plot()
        
    def update_plot(self):
        motor = int(self.motor_var.get())
        
        if not self.time_data[motor]:
            self.ax.set_title(f'Real-time PID Components - Motor {motor} (No data)')
            self.canvas.draw_idle()
            return
            
        x_data = list(range(len(self.time_data[motor])))
        
        self.p_line.set_data(x_data, self.p_data[motor])
        self.i_line.set_data(x_data, self.i_data[motor])
        self.d_line.set_data(x_data, self.d_data[motor])
        self.out_line.set_data(x_data, self.out_data[motor])
        
        # Adjust limits
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_title(f'Real-time PID Components - Motor {motor}')
        
        self.canvas.draw_idle()
        
    def update_display(self):
        # Update all motor value displays
        for motor in range(self.num_motors):
            values = self.current_values[motor]
            labels = self.value_labels[motor]
            
            labels[0].config(text=f"{motor}")  # Motor number
            labels[1].config(text=f"{values['error']:.2f}")  # Error
            labels[2].config(text=f"{values['p']:.2f}")  # P
            labels[3].config(text=f"{values['i']:.2f}")  # I
            labels[4].config(text=f"{values['d']:.2f}")  # D
            labels[5].config(text=f"{values['out']:.2f}")  # Output
            labels[6].config(text=f"{values['position']}")  # Position
            labels[7].config(text=f"{values['target']}")  # Target
        
        # Update plot for currently selected motor
        current_motor = int(self.motor_var.get())
        self.update_plot()
        
    def parse_pid_data(self, line):
        # Parse PID data line like: "M[1],Err=100.50,P=10.05,I=0.00,D=0.00,Out=10.05,PosT=1000,Pos=899,..."
        pattern = r'M\[(\d+)\],Err=([-\d.]+),P=([-\d.]+),I=([-\d.]+),D=([-\d.]+),Out=([-\d.]+),PosT=(\d+),Pos=(\d+)'
        match = re.search(pattern, line)
        
        if match:
            try:
                motor_num = int(match.group(1))
                if motor_num >= self.num_motors:
                    return None  # Skip if motor number out of range
                    
                error = float(match.group(2))
                p_val = float(match.group(3))
                i_val = float(match.group(4))
                d_val = float(match.group(5))
                out_val = float(match.group(6))
                target_pos = int(match.group(7))
                current_pos = int(match.group(8))
                
                return motor_num, error, p_val, i_val, d_val, out_val, target_pos, current_pos
            except ValueError:
                pass
        return None
        
    def serial_reader(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Check if it's PID data
                    pid_data = self.parse_pid_data(line)
                    if pid_data:
                        motor_num, error, p_val, i_val, d_val, out_val, target_pos, current_pos = pid_data
                        
                        # Update current values for this motor
                        self.current_values[motor_num].update({
                            'p': p_val, 'i': i_val, 'd': d_val, 'out': out_val,
                            'error': error, 'position': current_pos, 'target': target_pos
                        })
                        
                        # Update plot data for this motor
                        self.time_data[motor_num].append(len(self.time_data[motor_num]))
                        self.p_data[motor_num].append(p_val)
                        self.i_data[motor_num].append(i_val)
                        self.d_data[motor_num].append(d_val)
                        self.out_data[motor_num].append(out_val)
                        
                        # Limit data points
                        if len(self.time_data[motor_num]) > self.max_points:
                            self.time_data[motor_num].pop(0)
                            self.p_data[motor_num].pop(0)
                            self.i_data[motor_num].pop(0)
                            self.d_data[motor_num].pop(0)
                            self.out_data[motor_num].pop(0)
                        
                        # Update activity indicator
                        self.activity_vars[motor_num].set("●")
                        
                        # Schedule GUI update in main thread
                        self.root.after(0, self.update_display)
                    
                    # Print other messages (heartbeat, etc.)
                    elif any(msg in line for msg in ['HEARTBEAT', 'GUI_CONNECTED', 'PID_UPDATED']):
                        print(f"System: {line}")
                        
                    # Reset activity indicators after some time
                    self.root.after(2000, self.reset_activity_indicators)
                        
            except Exception as e:
                if self.running:
                    print(f"Serial read error: {e}")
                break
                
    def reset_activity_indicators(self):
        for i in range(self.num_motors):
            self.activity_vars[i].set("○")
                
    def setup_serial(self):
        # Close existing connection
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.running = False
            
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.running = True
            self.status_label.config(text=f"Connected to {self.serial_port}")
            
            # Start serial reader thread
            thread = threading.Thread(target=self.serial_reader, daemon=True)
            thread.start()
            
        except Exception as e:
            self.status_label.config(text=f"Connection failed: {e}")
            
    def run(self):
        # Start animation
        self.ani = FuncAnimation(self.fig, lambda x: None, interval=100, cache_frame_data=False)
        
        try:
            self.root.mainloop()
        finally:
            self.running = False
            if self.ser and self.ser.is_open:
                self.ser.close()

if __name__ == "__main__":
    # Update the serial port to match your system
    # Windows: 'COM3', Linux: '/dev/ttyUSB0', macOS: '/dev/tty.usbserial-*'
    plotter = PIDPlotter(serial_port='COM13', baud_rate=115200)
    plotter.run()