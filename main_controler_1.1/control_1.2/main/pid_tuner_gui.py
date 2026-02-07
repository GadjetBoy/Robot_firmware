import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

SERIAL_PORT = "COM4"  # <-- change to your ESP32 serial port
BAUD_RATE = 115200

class PIDTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Tuner GUI")
        self.root.geometry("1000x600")
        
        # PID values
        self.kp = tk.DoubleVar(value=0.1)
        self.ki = tk.DoubleVar(value=1.0)
        self.kd = tk.DoubleVar(value=0.025)
        self.connected = False
        self.running = True
        
        # Serial
        self.ser = None
        
        # Plot data buffers
        self.time_data = []
        self.error_data = []
        self.pwm_data = []
        self.pos_data = []
        self.start_time = time.time()

        # --- UI Layout ---
        control_frame = ttk.Frame(root, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(control_frame, text="PID Gains", font=("Arial", 14)).pack(pady=10)

        ttk.Label(control_frame, text="Kp").pack()
        ttk.Entry(control_frame, textvariable=self.kp).pack()

        ttk.Label(control_frame, text="Ki").pack()
        ttk.Entry(control_frame, textvariable=self.ki).pack()

        ttk.Label(control_frame, text="Kd").pack()
        ttk.Entry(control_frame, textvariable=self.kd).pack()

        ttk.Button(control_frame, text="Send PID", command=self.send_pid).pack(pady=10)
        ttk.Button(control_frame, text="Connect", command=self.connect_serial).pack(pady=5)
        ttk.Button(control_frame, text="Disconnect", command=self.disconnect_serial).pack(pady=5)
        ttk.Button(control_frame, text="Exit", command=self.exit_program).pack(pady=20)

        # --- Plot setup ---
        fig, self.ax = plt.subplots(figsize=(7, 4))
        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.error_line, = self.ax.plot([], [], label="Error")
        self.pwm_line, = self.ax.plot([], [], label="PWM")
        self.pos_line, = self.ax.plot([], [], label="Position")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Value")
        self.ax.legend()
        self.ax.grid(True)

        # Start background serial reader
        threading.Thread(target=self.serial_reader, daemon=True).start()
        self.update_plot()

    # === SERIAL HANDLING ===
    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.connected = True
            print("Connected to ESP32 on", SERIAL_PORT)
        except Exception as e:
            print("Connection failed:", e)

    def disconnect_serial(self):
        self.connected = False
        if self.ser:
            self.ser.close()

    def send_pid(self):
        if self.connected:
            msg = f"SET_PID,{self.kp.get():.3f},{self.ki.get():.3f},{self.kd.get():.3f}\n"
            self.ser.write(msg.encode())
            print("Sent:", msg.strip())

    def serial_reader(self):
        while self.running:
            if self.connected and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    if line.startswith("PIDLOG:"):
                        # Example: PIDLOG:0,err,P,I,D,OUT,dir,pos
                        parts = line.replace("PIDLOG:", "").split(",")
                        if len(parts) >= 8:
                            motor = int(parts[0])
                            error = float(parts[1])
                            p = float(parts[2])
                            i = float(parts[3])
                            d = float(parts[4])
                            out = float(parts[5])
                            pos = float(parts[7])
                            t = time.time() - self.start_time
                            self.time_data.append(t)
                            self.error_data.append(error)
                            self.pwm_data.append(out)
                            self.pos_data.append(pos)

                            if len(self.time_data) > 200:
                                self.time_data.pop(0)
                                self.error_data.pop(0)
                                self.pwm_data.pop(0)
                                self.pos_data.pop(0)
                except Exception:
                    pass
            time.sleep(0.02)

    # === PLOT UPDATE ===
    def update_plot(self):
        self.error_line.set_data(self.time_data, self.error_data)
        self.pwm_line.set_data(self.time_data, self.pwm_data)
        self.pos_line.set_data(self.time_data, self.pos_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
        self.root.after(100, self.update_plot)

    def exit_program(self):
        self.running = False
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerGUI(root)
    root.mainloop()
