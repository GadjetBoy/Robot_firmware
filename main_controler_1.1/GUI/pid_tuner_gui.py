import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import queue
import re

SERIAL_PORT = "COM13"
BAUD_RATE = 115200

class PIDTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Tuner GUI - 8 Motors")
        self.root.geometry("1400x900")
        
        # PID values
        self.kp = tk.DoubleVar(value=0.170)
        self.ki = tk.DoubleVar(value=0.001)
        self.kd = tk.DoubleVar(value=0.020)
        
        # Flags
        self.connected = False
        self.running = True
        self.after_id = None
        
        # Serial
        self.ser = None
        self.data_queue = queue.Queue()
        
        # Plot data buffers
        self.motor_data = {
            i: {
                'time': [],
                'error': [],
                'output': [],
                'position': [],
                'target_position': [],
                'pwm': [],
                'active': False,
                'last_update': 0
            } for i in range(8)
        }
        self.start_time = time.time()

        # Colors for each motor
        self.motor_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        self.motor_names = [f'Motor {i+1}' for i in range(8)]

        # UI setup
        control_frame = ttk.Frame(root, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(control_frame, text="PID Gains", font=("Arial", 14)).pack(pady=10)
        ttk.Label(control_frame, text="Kp").pack()
        ttk.Entry(control_frame, textvariable=self.kp).pack()
        ttk.Label(control_frame, text="Ki").pack()
        ttk.Entry(control_frame, textvariable=self.ki).pack()
        ttk.Label(control_frame, text="Kd").pack()
        ttk.Entry(control_frame, textvariable=self.kd).pack()

        ttk.Button(control_frame, text="Send PID to All Motors", command=self.send_pid).pack(pady=10)
        ttk.Button(control_frame, text="Connect", command=self.connect_serial).pack(pady=5)
        ttk.Button(control_frame, text="Disconnect", command=self.disconnect_serial).pack(pady=5)
        
        # Test button
        ttk.Button(control_frame, text="Test Connection", command=self.test_connection).pack(pady=5)
        ttk.Button(control_frame, text="Clear Plot", command=self.clear_plot).pack(pady=5)
        
        # Debug frame
        debug_frame = ttk.LabelFrame(control_frame, text="Debug", padding=5)
        debug_frame.pack(pady=10, fill=tk.X)
        self.debug_text = tk.Text(debug_frame, height=10, width=35)
        self.debug_text.pack(fill=tk.BOTH, expand=True)
        
        ttk.Button(control_frame, text="Exit", command=self.exit_program).pack(pady=20)

        # Status label
        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red")
        self.status_label.pack(pady=10)

        # Motor activity indicators
        ttk.Label(control_frame, text="Motor Status", font=("Arial", 12)).pack(pady=(20,5))
        self.motor_status_labels = []
        for i in range(8):
            frame = ttk.Frame(control_frame)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=f"M{i+1}:", width=6).pack(side=tk.LEFT)
            status_label = ttk.Label(frame, text="OFF", foreground="red", width=8)
            status_label.pack(side=tk.LEFT)
            self.motor_status_labels.append(status_label)

        # Plot setup
        plot_frame = ttk.Frame(root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        self.toolbar.update()
        
        # Initialize lines for all motors
        self.error_lines = []
        self.output_lines = []
        self.position_lines = []
        
        for i in range(8):
            error_line, = self.ax1.plot([], [], color=self.motor_colors[i], 
                                      label=self.motor_names[i], linewidth=1.5, alpha=0.8)
            output_line, = self.ax2.plot([], [], color=self.motor_colors[i], 
                                       label=self.motor_names[i], linewidth=1.5, alpha=0.8)
            position_line, = self.ax3.plot([], [], color=self.motor_colors[i], 
                                         label=self.motor_names[i], linewidth=1.5, alpha=0.8)
            
            self.error_lines.append(error_line)
            self.output_lines.append(output_line)
            self.position_lines.append(position_line)
        
        # Configure subplots
        self.ax1.set_title("PID Error")
        self.ax1.set_ylabel("Error")
        self.ax1.legend(loc='upper right', fontsize=8)
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_title("PID Output")
        self.ax2.set_ylabel("Output")
        self.ax2.legend(loc='upper right', fontsize=8)
        self.ax2.grid(True, alpha=0.3)
        
        self.ax3.set_title("Motor Position")
        self.ax3.set_ylabel("Position")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.legend(loc='upper right', fontsize=8)
        self.ax3.grid(True, alpha=0.3)
        
        self.fig.tight_layout()

        # Start threads
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()
        
        # Start plot updates
        self.update_plot()

        self.root.protocol("WM_DELETE_WINDOW", self.exit_program)

    def debug_log(self, message):
        if not self.running:
            return
        try:
            self.debug_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
            self.debug_text.see(tk.END)
            if self.debug_text.get('1.0', tk.END).count('\n') > 50:
                self.debug_text.delete('1.0', '2.0')
        except tk.TclError:
            pass

    def test_connection(self):
        """Test if we can send and receive data"""
        if not self.connected:
            messagebox.showwarning("Warning", "Not connected to ESP32")
            return
            
        # Send test message
        test_msg = "TEST: Hello from GUI\n"
        try:
            self.ser.write(test_msg.encode())
            self.debug_log("Sent test message to ESP32")
        except Exception as e:
            self.debug_log(f"Failed to send test: {e}")

    def parse_data_line(self, line):
        """Improved parsing that shows what's actually being received"""
        # First, log what we're receiving for debugging
        if any(x in line for x in ['GUI_CONNECTED', 'HEARTBEAT', 'TEST']):
            self.debug_log(f"System: {line.strip()}")
            return None
            
        # Try to parse motor data
        try:
            # Must start with M[ and have proper structure
            if not line.startswith("M[") or line.count(',') < 8:
                return None
                
            # Extract motor index
            motor_match = re.search(r'M\[(\d+)\]', line)
            if not motor_match:
                return None
                
            motor_index = int(motor_match.group(1)) - 1
            
            # Extract values
            def safe_extract(pattern, default=0.0):
                match = re.search(pattern, line)
                return float(match.group(1)) if match else default
            
            error = safe_extract(r'Err=([-\d.]+)')
            output = safe_extract(r'Out=([-\d.]+)')
            position = safe_extract(r'Pos=([-\d.]+)')
            target_position = safe_extract(r'PosT=([-\d.]+)')
            pwm = safe_extract(r'PWM=([-\d.]+)')
            
            # Extract direction
            dir_match = re.search(r'Dir=([-\d]+)', line)
            direction = int(dir_match.group(1)) if dir_match else 1
            
            return (motor_index, error, output, position, target_position, pwm, direction)
                        
        except Exception as e:
            # Log parsing errors occasionally
            if "M[" in line and len(line) > 10:
                self.debug_log(f"Parse error: {line[:50]}...")
            return None

    def serial_reader(self):
        buffer = ""
        valid_data_count = 0
        total_line_count = 0
        
        while self.running:
            if self.connected and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                        buffer += data
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            total_line_count += 1
                            
                            if line:
                                result = self.parse_data_line(line)
                                if result:
                                    (motor_index, error, output, position, 
                                     target_position, pwm, direction) = result
                                    
                                    if 0 <= motor_index < 8:
                                        t = time.time() - self.start_time
                                        self.data_queue.put((motor_index, t, error, output, 
                                                           position, target_position, pwm))
                                        if self.running:
                                            self.root.after(0, self.update_motor_status, motor_index, True)
                                        
                                        valid_data_count += 1
                                        
                                        # Log first few successes
                                        if valid_data_count <= 5:
                                            self.debug_log(f"✓ Motor {motor_index+1} data received")
                                
                                # Log statistics
                                if total_line_count % 100 == 0:
                                    success_rate = (valid_data_count / total_line_count) * 100
                                    self.debug_log(f"Data: {success_rate:.1f}% valid ({valid_data_count}/{total_line_count})")
                                    
                except Exception as e:
                    if self.running:
                        self.debug_log(f"Serial error: {e}")
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def update_motor_status(self, motor_index, active):
        if not self.running:
            return
        try:
            if active:
                self.motor_status_labels[motor_index].config(text="ACTIVE", foreground="green")
            else:
                self.motor_status_labels[motor_index].config(text="OFF", foreground="red")
        except tk.TclError:
            pass

    def update_plot(self):
        if not self.running:
            return
            
        try:
            # Process data from queue
            data_processed = False
            while not self.data_queue.empty():
                try:
                    (motor_index, t, error, output, 
                     position, target_position, pwm) = self.data_queue.get_nowait()
                    
                    motor = self.motor_data[motor_index]
                    motor['time'].append(t)
                    motor['error'].append(error)
                    motor['output'].append(output)
                    motor['position'].append(position)
                    motor['target_position'].append(target_position)
                    motor['pwm'].append(pwm)
                    motor['active'] = True
                    motor['last_update'] = time.time()
                    data_processed = True
                    
                except queue.Empty:
                    break

            # Update plots for all motors
            for i in range(8):
                motor = self.motor_data[i]
                
                if motor['time'] and motor['error']:
                    self.error_lines[i].set_data(motor['time'], motor['error'])
                    self.output_lines[i].set_data(motor['time'], motor['output'])
                    self.position_lines[i].set_data(motor['time'], motor['position'])
                    
                    if time.time() - motor['last_update'] > 5:
                        motor['active'] = False
                        if self.running:
                            self.root.after(0, self.update_motor_status, i, False)

            # Auto-scale if needed
            if data_processed:
                for ax in [self.ax1, self.ax2, self.ax3]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas.draw()

            if self.running:
                self.after_id = self.root.after(100, self.update_plot)
                
        except Exception as e:
            if self.running:
                self.debug_log(f"Plot update error: {e}")

    def connect_serial(self):
        if self.connected:
            return
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.debug_log(f"Connected to {SERIAL_PORT}")
            self.debug_log("Waiting for ESP32 data...")
        except Exception as e:
            messagebox.showerror("Error", f"Connection failed: {e}")
            self.debug_log(f"Connection failed: {e}")

    def disconnect_serial(self):
        self.connected = False
        if self.ser:
            self.ser.close()
            self.ser = None
        self.status_label.config(text="Disconnected", foreground="red")
        self.debug_log("Disconnected")

    def send_pid(self):
        if not self.connected:
            messagebox.showwarning("Warning", "Not connected to ESP32")
            return
        msg = f"SET_PID,{self.kp.get():.3f},{self.ki.get():.3f},{self.kd.get():.3f}\n"
        try:
            self.ser.write(msg.encode())
            self.debug_log(f"Sent PID: Kp={self.kp.get():.3f}, Ki={self.ki.get():.3f}, Kd={self.kd.get():.3f}")
        except Exception as e:
            self.debug_log(f"Failed to send PID: {e}")

    def clear_plot(self):
        if not self.running:
            return
        for i in range(8):
            self.motor_data[i]['time'].clear()
            self.motor_data[i]['error'].clear()
            self.motor_data[i]['output'].clear()
            self.motor_data[i]['position'].clear()
            self.motor_data[i]['target_position'].clear()
            self.motor_data[i]['pwm'].clear()
            self.motor_data[i]['active'] = False
        self.start_time = time.time()
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw()
        self.debug_log("Plot cleared")

    def exit_program(self):
        self.running = False
        if hasattr(self, 'after_id') and self.after_id:
            try:
                self.root.after_cancel(self.after_id)
            except:
                pass
        self.disconnect_serial()
        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.exit_program()