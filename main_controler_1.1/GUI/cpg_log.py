import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import re  # For parsing

# Adjust port/baud as needed (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux/Mac)
ser = serial.Serial('COM13', 115200, timeout=1)  # Change 'COM3' to your ESP32 port
data_buffer = deque(maxlen=1000)  # Rolling buffer for ~10s at 100 Hz

fig, ax = plt.subplots(figsize=(10, 6))
lines = []  # One line per oscillator
# FIXED: Valid colors only (no 'o' marker)
colors = ['b', 'orange', 'g', 'r', 'm', 'y', 'c', 'k']  # Blue, Orange, Green, Red, Magenta, Yellow, Cyan, Black
for i in range(8):
    line, = ax.plot([], [], color=colors[i], label=f'Osc {i}', linewidth=1.5)
    lines.append(line)

ax.set_title('Live CPG Outputs')
ax.set_xlabel('Time Steps')
ax.set_ylabel('Output Value')
ax.set_ylim(-70000, 70000)  # Adjust based on your amps (±65k for knees)
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)

def animate(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('CPG:'):
            # Parse: CPG:val1,val2,...,val8 (robust to extra chars)
            match = re.search(r'CPG:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?(?:,\s*[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?){7})', line)
            if match:
                vals_str = match.group(1)
                vals = [float(v) for v in vals_str.split(',')]
                if len(vals) == 8:
                    data_buffer.append(vals)
                    # Convert to arrays for plotting
                    times = np.arange(len(data_buffer))
                    for i in range(8):
                        y = np.array([d[i] for d in data_buffer])
                        lines[i].set_data(times, y)
                    # Auto-scale x if buffer grows
                    ax.set_xlim(0, max(100, len(times) - 1))
    except (ValueError, IndexError, serial.SerialException):
        pass  # Skip bad lines or serial hiccups
    return lines

ani = animation.FuncAnimation(fig, animate, interval=10, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()

# Clean up on exit
ser.close()