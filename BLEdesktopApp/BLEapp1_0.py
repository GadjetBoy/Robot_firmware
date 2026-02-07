import os
os.environ['BLEAK_LOGGING'] = '1'  # Enable Bleak logs for debug
import tkinter as tk
from tkinter import ttk, messagebox, font
import struct
import threading
import asyncio
from bleak import BleakScanner, BleakClient
import sys
import queue
import time

# Fix for Windows BLE scanning (MTA threading model)
if sys.platform == "win32":
    sys.coinit_flags = 0  # Force Multi-Threaded Apartment before imports

# BLE UUIDs (same as your Flutter app)
SERVICE_UUID = "cb341ded-25f3-244f-11ac-7ebc1a247a86"
RX_CHAR_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"

# Modern color scheme
COLORS = {
    "bg_dark": "#121212",
    "bg_medium": "#1e1e1e",
    "bg_light": "#2d2d2d",
    "bg_card": "#252525",
    "primary": "#007acc",
    "primary_light": "#3399ff",
    "secondary": "#00b894",
    "accent": "#ff6b6b",
    "warning": "#f39c12",
    "success": "#27ae60",
    "danger": "#e74c3c",
    "text_primary": "#ffffff",
    "text_secondary": "#b0b0b0",
    "text_muted": "#888888",
    "border": "#404040",
    "hover": "#3d3d3d"
}

class ModernButton(tk.Button):
    """Custom modern button with hover effects"""
    def __init__(self, parent, **kwargs):
        self.bg_color = kwargs.pop('bg_color', COLORS["primary"])
        self.hover_color = kwargs.pop('hover_color', COLORS["primary_light"])
        self.fg_color = kwargs.pop('fg_color', COLORS["text_primary"])

        button_font = kwargs.pop('font', ('Segoe UI', 10, 'bold'))
        button_padx = kwargs.pop('padx', 15)
        button_pady = kwargs.pop('pady', 8)

        super().__init__(parent,
                         bg=self.bg_color,
                         fg=self.fg_color,
                         activebackground=self.hover_color,
                         activeforeground=self.fg_color,
                         relief=tk.FLAT,
                         cursor="hand2",
                         font=button_font,
                         padx=button_padx,
                         pady=button_pady,
                         **kwargs)

        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        self.config(bg=self.hover_color)

    def on_leave(self, e):
        self.config(bg=self.bg_color)


class RobotController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 Robot BLE Controller Pro")
        self.root.geometry("900x1000")
        self.root.configure(bg=COLORS["bg_dark"])

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.device_address = None
        self.client = None
        self.connected = False
        self.float_values = [0.0] * 16
        self.control_byte = 0
        self.control_buttons = {}
        self.mode_buttons = {}       # NEW: for mode selection
        self.devices = []

        self.ui_queue = queue.Queue()
        self.scanning = False
        self.stop_scan_event = threading.Event()

        self.float_entries = [None] * 16

        self.bytes_sent = 0
        self.packets_sent = 0
        self.connection_start_time = None

        self.fonts = {
            'title': ('Segoe UI', 16, 'bold'),
            'heading': ('Segoe UI', 12, 'bold'),
            'subheading': ('Segoe UI', 10, 'bold'),
            'body': ('Segoe UI', 10),
            'mono': ('Consolas', 9),
            'small': ('Segoe UI', 8)
        }

        self.setup_ui()
        self.root.after(100, self.process_queue)
        self.center_window()

    def center_window(self):
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')

    def setup_ui(self):
        # Header
        header_frame = tk.Frame(self.root, bg=COLORS["bg_card"], height=80)
        header_frame.pack(fill=tk.X, padx=0, pady=0)
        header_frame.pack_propagate(False)

        title_frame = tk.Frame(header_frame, bg=COLORS["bg_card"])
        title_frame.pack(side=tk.LEFT, padx=25, pady=20)

        tk.Label(title_frame, text="🤖 Robot BLE Controller Pro",
                 font=self.fonts['title'], fg=COLORS["primary_light"],
                 bg=COLORS["bg_card"]).pack(side=tk.LEFT)

        tk.Label(title_frame, text="v2.0", font=self.fonts['small'],
                 fg=COLORS["text_muted"], bg=COLORS["bg_card"],
                 padx=5).pack(side=tk.LEFT, padx=10)

        self.status_frame = tk.Frame(header_frame, bg=COLORS["bg_card"])
        self.status_frame.pack(side=tk.RIGHT, padx=25, pady=20)

        self.status_indicator = tk.Label(self.status_frame, text="●",
                                        font=('Arial', 20), fg=COLORS["danger"],
                                        bg=COLORS["bg_card"])
        self.status_indicator.pack(side=tk.LEFT, padx=5)

        self.status_label = tk.Label(self.status_frame, text="Disconnected",
                                    font=self.fonts['heading'], fg=COLORS["text_primary"],
                                    bg=COLORS["bg_card"])
        self.status_label.pack(side=tk.LEFT)

        self.connection_info = tk.Label(self.status_frame, text="No device connected",
                                       font=self.fonts['small'], fg=COLORS["text_muted"],
                                       bg=COLORS["bg_card"])
        self.connection_info.pack(side=tk.LEFT, padx=10)

        # Main container + notebook
        main_container = tk.Frame(self.root, bg=COLORS["bg_medium"])
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        self.notebook = ttk.Notebook(main_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TNotebook', background=COLORS["bg_medium"], borderwidth=0)
        style.configure('TNotebook.Tab', background=COLORS["bg_light"],
                        foreground=COLORS["text_primary"], padding=[15, 5],
                        font=self.fonts['body'])
        style.map('TNotebook.Tab', background=[('selected', COLORS["primary"])],
                  foreground=[('selected', COLORS["text_primary"])])

        self.setup_connection_tab()
        self.setup_controls_tab()
        self.setup_log_tab()

        # Status bar
        status_bar = tk.Frame(self.root, bg=COLORS["bg_card"], height=30)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)
        status_bar.pack_propagate(False)

        self.packet_info = tk.Label(status_bar,
                                   text="Ready | Packet: 68 bytes (1B + 1B + 64B + 2B)",
                                   font=self.fonts['small'], fg=COLORS["text_muted"],
                                   bg=COLORS["bg_card"])
        self.packet_info.pack(side=tk.LEFT, padx=15)

        self.bytes_sent_label = tk.Label(status_bar, text="Bytes sent: 0",
                                        font=self.fonts['small'], fg=COLORS["text_muted"],
                                        bg=COLORS["bg_card"])
        self.bytes_sent_label.pack(side=tk.RIGHT, padx=15)

    def setup_connection_tab(self):
        connection_tab = tk.Frame(self.notebook, bg=COLORS["bg_medium"])
        self.notebook.add(connection_tab, text="📶 Connection")

        control_frame = tk.LabelFrame(connection_tab, text=" Device Management ",
                                     font=self.fonts['heading'], bg=COLORS["bg_card"],
                                     fg=COLORS["primary_light"], relief=tk.FLAT,
                                     borderwidth=2, padx=20, pady=20)
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        button_grid = tk.Frame(control_frame, bg=COLORS["bg_card"])
        button_grid.pack()

        self.scan_button = ModernButton(button_grid, text="🔍 Scan Devices",
                                       bg_color=COLORS["primary"],
                                       command=self.start_scan_thread, width=15)
        self.scan_button.grid(row=0, column=0, padx=5, pady=5)

        self.stop_btn = ModernButton(button_grid, text="⏹ Stop Scan",
                                    bg_color=COLORS["danger"], command=self.stop_scan,
                                    state='disabled', width=15)
        self.stop_btn.grid(row=0, column=1, padx=5, pady=5)

        self.disconnect_btn = ModernButton(button_grid, text="🔌 Disconnect",
                                          bg_color=COLORS["warning"],
                                          command=self.disconnect_device,
                                          state='disabled', width=15)
        self.disconnect_btn.grid(row=0, column=2, padx=5, pady=5)

        device_frame = tk.LabelFrame(connection_tab, text=" Available Devices ",
                                    font=self.fonts['heading'], bg=COLORS["bg_card"],
                                    fg=COLORS["secondary"], relief=tk.FLAT,
                                    borderwidth=2, padx=20, pady=20)
        device_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        listbox_container = tk.Frame(device_frame, bg=COLORS["bg_card"])
        listbox_container.pack(fill=tk.BOTH, expand=True)

        scrollbar = tk.Scrollbar(listbox_container, bg=COLORS["bg_light"],
                                troughcolor=COLORS["bg_medium"],
                                activebackground=COLORS["primary"])
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.device_listbox = tk.Listbox(listbox_container, bg=COLORS["bg_light"],
                                        fg=COLORS["text_primary"], font=self.fonts['mono'],
                                        height=10, selectbackground=COLORS["primary"],
                                        selectforeground=COLORS["text_primary"],
                                        activestyle='none', borderwidth=0,
                                        highlightthickness=0)
        self.device_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.device_listbox.bind('<<ListboxSelect>>', self.on_device_select)
        self.device_listbox.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.device_listbox.yview)

        self.connect_button = ModernButton(device_frame,
                                          text="✅ Connect to Selected Device",
                                          bg_color=COLORS["success"],
                                          command=self.connect_selected,
                                          state='disabled')
        self.connect_button.pack(pady=10)

    def setup_controls_tab(self):
        controls_tab = tk.Frame(self.notebook, bg=COLORS["bg_medium"])
        self.notebook.add(controls_tab, text="🎮 Controls")

        top_frame = tk.Frame(controls_tab, bg=COLORS["bg_medium"])
        top_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        float_frame = tk.LabelFrame(top_frame, text=" Float Parameters (16 values) ",
                                   font=self.fonts['heading'], bg=COLORS["bg_card"],
                                   fg=COLORS["accent"], relief=tk.FLAT,
                                   borderwidth=2, padx=20, pady=20)
        float_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        for i in range(16):
            row = i % 4
            col = i // 4
            frame = tk.Frame(float_frame, bg=COLORS["bg_card"])
            frame.grid(row=row, column=col, padx=10, pady=8, sticky='w')

            label_bg = self.get_gradient_color(i, 16)
            tk.Label(frame, text=f"F{i+1:02d}:", font=self.fonts['body'],
                     fg=COLORS["text_primary"], bg=label_bg,
                     padx=8, pady=3, relief=tk.RAISED,
                     borderwidth=1).pack(side=tk.LEFT)

            entry = tk.Entry(frame, bg=COLORS["bg_light"], fg=COLORS["text_primary"],
                             font=self.fonts['body'], width=12,
                             insertbackground=COLORS["primary_light"],
                             relief=tk.FLAT, borderwidth=1)
            entry.insert(0, "0.0")
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<FocusIn>", lambda e, w=entry: w.config(bg=COLORS["bg_medium"]))
            entry.bind("<FocusOut>", lambda e, w=entry: w.config(bg=COLORS["bg_light"]))

            value_label = tk.Label(frame, text="0.00", font=self.fonts['small'],
                                  fg=COLORS["text_muted"], bg=COLORS["bg_card"], width=6)
            value_label.pack(side=tk.LEFT, padx=5)

            entry.value_label = value_label
            entry.bind("<KeyRelease>", lambda e, idx=i: self.update_value_label(idx))
            self.float_entries[i] = entry

        bottom_frame = tk.Frame(controls_tab, bg=COLORS["bg_medium"])
        bottom_frame.pack(fill=tk.X, padx=10, pady=10)

                # Left side: Movement + Mode (using grid for reliable vertical stacking)
        left_container = tk.Frame(bottom_frame, bg=COLORS["bg_medium"])
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Configure grid weights
        left_container.grid_rowconfigure(0, weight=1)
        left_container.grid_rowconfigure(1, weight=0)
        left_container.grid_columnconfigure(0, weight=1)

        # Robot Movement controls
        control_buttons_frame = tk.LabelFrame(left_container,
                                             text=" Robot Movement ",
                                             font=self.fonts['heading'],
                                             bg=COLORS["bg_card"],
                                             fg=COLORS["primary_light"],
                                             relief=tk.FLAT,
                                             borderwidth=2,
                                             padx=20,
                                             pady=20)
        control_buttons_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=(0, 10))
        self.control_frame = control_buttons_frame
        self.show_controls(disabled=True)

        # Robot Mode selection
        mode_frame = tk.LabelFrame(left_container,
                                   text=" Robot Mode ",
                                   font=self.fonts['heading'],
                                   bg=COLORS["bg_card"],
                                   fg=COLORS["secondary"],
                                   relief=tk.FLAT,
                                   borderwidth=2,
                                   padx=20,
                                   pady=20)
        mode_frame.grid(row=1, column=0, sticky='ew', padx=5, pady=(0, 10))

        mode_grid = tk.Frame(mode_frame, bg=COLORS["bg_card"])
        mode_grid.pack(padx=10, pady=10)

        modes = [
            ("Idle",     0, COLORS["text_muted"]),
            ("Standby",  1, COLORS["primary"]),
            ("Crawl",    2, COLORS["secondary"]),
            ("Turtle",   3, COLORS["warning"]),
            ("Creep",    4, COLORS["accent"])
        ]

        self.mode_buttons = {}
        for i, (name, value, base_color) in enumerate(modes):
            btn = ModernButton(mode_grid,
                               text=name,
                               bg_color=base_color,
                               command=lambda v=value, n=name: self.select_mode(v, n),
                               width=12,
                               pady=8)
            btn.grid(row=i // 3, column=i % 3, padx=8, pady=6, sticky='ew')
            self.mode_buttons[value] = btn

        for c in range(3):
              mode_grid.grid_columnconfigure(c, weight=1)

        # Default to Idle
        self.select_mode(0, "Idle", silent=True)
        for c in range(3):
              mode_grid.grid_columnconfigure(c, weight=1)

        # Default to Idle
        self.select_mode(0, "Idle", silent=True)

        for c in range(3):
              mode_grid.grid_columnconfigure(c, weight=1)

        # Default to Idle
        self.select_mode(0, "Idle", silent=True)

        # Default to Idle mode
        self.select_mode(0, "Idle", silent=True)
        # Right side: Presets + Send
        right_frame = tk.Frame(bottom_frame, bg=COLORS["bg_medium"])
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        presets_frame = tk.LabelFrame(right_frame, text=" Quick Presets ",
                                     font=self.fonts['heading'], bg=COLORS["bg_card"],
                                     fg=COLORS["warning"], relief=tk.FLAT,
                                     borderwidth=2, padx=20, pady=20)
        presets_frame.pack(fill=tk.X, pady=(0, 10))

        presets_grid = tk.Frame(presets_frame, bg=COLORS["bg_card"])
        presets_grid.pack()

        preset_configs = [
            ("Set All Zero", self.set_all_zero, COLORS["text_muted"]),
            ("Test Pattern", self.set_test_pattern, COLORS["primary"]),
            ("Motor Control", self.set_motor_control, COLORS["secondary"]),
            ("PID Values", self.set_pid_values, COLORS["accent"])
        ]

        for i, (text, command, color) in enumerate(preset_configs):
            btn = ModernButton(presets_grid, text=text, bg_color=color,
                               command=command, width=12)
            btn.grid(row=i//2, column=i%2, padx=5, pady=5, sticky='ew')

        send_frame = tk.LabelFrame(right_frame, text=" Send Control ",
                                  font=self.fonts['heading'], bg=COLORS["bg_card"],
                                  fg=COLORS["success"], relief=tk.FLAT,
                                  borderwidth=2, padx=20, pady=20)
        send_frame.pack(fill=tk.X)

        self.send_button = ModernButton(send_frame, text="🚀 SEND PACKET",
                                       bg_color=COLORS["success"],
                                       fg_color=COLORS["bg_dark"],
                                       command=self.send_packet,
                                       state='disabled',
                                       font=('Segoe UI', 11, 'bold'),
                                       padx=30, pady=12)
        self.send_button.pack(pady=5)

        self.packet_status = tk.Label(send_frame, text="Ready to send",
                                     font=self.fonts['body'], fg=COLORS["text_muted"],
                                     bg=COLORS["bg_card"])
        self.packet_status.pack()

    def select_mode(self, mode_value, mode_name, silent=False):
        """Set robot mode and update button highlight"""
        if not self.connected and not silent:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return

        self.control_byte = mode_value

        # Highlight active button
        for val, btn in self.mode_buttons.items():
            if val == mode_value:
                btn.config(bg=COLORS["success"], fg=COLORS["bg_dark"])
            else:
                colors = {0: COLORS["text_muted"], 1: COLORS["primary"],
                          2: COLORS["secondary"], 3: COLORS["warning"],
                          4: COLORS["accent"]}
                btn.config(bg=colors[val], fg=COLORS["text_primary"])

        if not silent:
            self.log(f"Mode changed to: {mode_name} ({mode_value})", tag="info")
            self.send_packet()

    def setup_log_tab(self):
        log_tab = tk.Frame(self.notebook, bg=COLORS["bg_medium"])
        self.notebook.add(log_tab, text="📊 Logs & Stats")

        stats_frame = tk.LabelFrame(log_tab, text=" Connection Statistics ",
                                   font=self.fonts['heading'], bg=COLORS["bg_card"],
                                   fg=COLORS["primary_light"], relief=tk.FLAT,
                                   borderwidth=2, padx=20, pady=20)
        stats_frame.pack(fill=tk.X, padx=10, pady=10)

        stats_grid = tk.Frame(stats_frame, bg=COLORS["bg_card"])
        stats_grid.pack()

        stats_data = [
            ("Connected Device:", "None", "device_label"),
            ("Packets Sent:", "0", "packets_label"),
            ("Bytes Transferred:", "0 B", "bytes_label"),
            ("Connection Time:", "00:00:00", "time_label"),
            ("Signal Strength:", "- dBm", "rssi_label"),
            ("Last Update:", "Never", "update_label")
        ]

        self.stats_labels = {}
        for i, (label, value, key) in enumerate(stats_data):
            tk.Label(stats_grid, text=label, font=self.fonts['body'],
                     fg=COLORS["text_secondary"], bg=COLORS["bg_card"]
                     ).grid(row=i, column=0, sticky='w', padx=(0, 10), pady=3)
            val_lbl = tk.Label(stats_grid, text=value, font=self.fonts['body'],
                               fg=COLORS["text_primary"], bg=COLORS["bg_card"])
            val_lbl.grid(row=i, column=1, sticky='w', pady=3)
            self.stats_labels[key] = val_lbl

        log_frame = tk.LabelFrame(log_tab, text=" Event Log ",
                                 font=self.fonts['heading'], bg=COLORS["bg_card"],
                                 fg=COLORS["accent"], relief=tk.FLAT,
                                 borderwidth=2, padx=20, pady=20)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        log_controls = tk.Frame(log_frame, bg=COLORS["bg_card"])
        log_controls.pack(fill=tk.X, pady=(0, 10))

        ModernButton(log_controls, text="Clear Log", bg_color=COLORS["danger"],
                     command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ModernButton(log_controls, text="Export Log", bg_color=COLORS["primary"],
                     command=self.export_log).pack(side=tk.LEFT, padx=5)
        ModernButton(log_controls, text="Copy to Clipboard", bg_color=COLORS["secondary"],
                     command=self.copy_log).pack(side=tk.LEFT, padx=5)

        log_container = tk.Frame(log_frame, bg=COLORS["bg_card"])
        log_container.pack(fill=tk.BOTH, expand=True)

        scrollbar = tk.Scrollbar(log_container, bg=COLORS["bg_light"],
                                troughcolor=COLORS["bg_medium"],
                                activebackground=COLORS["primary"])
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.log_text = tk.Text(log_container, height=15, bg=COLORS["bg_light"],
                               fg=COLORS["text_primary"], font=self.fonts['mono'],
                               wrap=tk.WORD, borderwidth=0, highlightthickness=0,
                               padx=10, pady=10)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.log_text.yview)

        self.log_text.tag_config("success", foreground=COLORS["success"])
        self.log_text.tag_config("error", foreground=COLORS["danger"])
        self.log_text.tag_config("warning", foreground=COLORS["warning"])
        self.log_text.tag_config("info", foreground=COLORS["primary_light"])

    def get_gradient_color(self, index, total):
        r1, g1, b1 = 255, 107, 107
        r2, g2, b2 = 0, 122, 204
        ratio = index / (total - 1) if total > 1 else 0
        r = int(r1 + (r2 - r1) * ratio)
        g = int(g1 + (g2 - g1) * ratio)
        b = int(b1 + (b2 - b1) * ratio)
        return f'#{r:02x}{g:02x}{b:02x}'

    def update_value_label(self, index):
        try:
            value = float(self.float_entries[index].get())
            self.float_entries[index].value_label.config(
                text=f"{value:.2f}",
                fg=COLORS["success"] if value != 0 else COLORS["text_muted"]
            )
        except:
            self.float_entries[index].value_label.config(text="Err", fg=COLORS["danger"])

    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
        self.log("Log cleared", tag="info")

    def export_log(self):
        try:
            filename = f"robot_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
            with open(filename, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"Log exported to {filename}", tag="success")
        except Exception as e:
            self.log(f"Export failed: {e}", tag="error")

    def copy_log(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.log_text.get(1.0, tk.END))
        self.log("Log copied to clipboard", tag="info")

    def log(self, message, tag="info"):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.ui_queue.put(("log", (log_entry, tag)))
        print(message)

    def process_queue(self):
        try:
            while True:
                msg_type, data = self.ui_queue.get_nowait()
                if msg_type == "log":
                    message, tag = data
                    self.log_text.insert(tk.END, f"{message}\n", tag)
                    self.log_text.see(tk.END)
                    if hasattr(self, 'stats_labels'):
                        self.stats_labels['update_label'].config(text=time.strftime("%H:%M:%S"))
                elif msg_type == "update_list":
                    self.device_listbox.delete(0, tk.END)
                    for device in data:
                        self.device_listbox.insert(tk.END, device)
                elif msg_type == "update_status":
                    text, color = data
                    self.status_label.config(text=text)
                    self.status_indicator.config(fg=color)
                elif msg_type == "update_info":
                    self.connection_info.config(text=data)
                elif msg_type == "enable_button":
                    button, state = data
                    button.config(state=state)
                elif msg_type == "show_error":
                    messagebox.showerror("Error", data)
                elif msg_type == "packet_sent":
                    self.packet_status.config(text=data[0], fg=data[1])
                    self.root.after(2000, lambda: self.packet_status.config(
                        text="Ready to send", fg=COLORS["text_muted"]))
                elif msg_type == "update_stats":
                    key, value = data
                    if key in self.stats_labels:
                        self.stats_labels[key].config(text=value)
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)

    # === All other methods remain unchanged (scan, connect, send, etc.) ===
    # (For brevity, only the changed parts are shown above, but the rest is identical to your original code)

    def start_scan_thread(self):
        if self.scanning:
            return
        self.scanning = True
        self.stop_scan_event.clear()
        self.devices = []
        self.device_listbox.delete(0, tk.END)

        self.ui_queue.put(("update_status", ["Scanning...", COLORS["warning"]]))
        self.ui_queue.put(("update_info", "Searching for BLE devices..."))
        self.ui_queue.put(("enable_button", [self.scan_button, "disabled"]))
        self.ui_queue.put(("enable_button", [self.stop_btn, "normal"]))
        self.ui_queue.put(("enable_button", [self.disconnect_btn, "disabled"]))

        self.log("Starting BLE device scan...", tag="info")
        scan_thread = threading.Thread(target=self.scan_ble_devices, daemon=True)
        scan_thread.start()

    def scan_ble_devices(self):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def scan():
                scanner = BleakScanner()
                devices = await scanner.discover(timeout=8.0)
                return devices

            discovered = loop.run_until_complete(scan())
            loop.close()

            device_strings = []
            for device in discovered:
                name = device.name or "Unknown"
                rssi = device.rssi if hasattr(device, 'rssi') and device.rssi else -100
                display = f"{name[:25]:25} | {device.address} | RSSI: {rssi:3}"
                device_strings.append(display)
                self.devices.append(device)
                self.log(f"Found: {name} ({device.address}) RSSI: {rssi}", tag="info")

            self.ui_queue.put(("update_list", device_strings))
            if discovered:
                self.ui_queue.put(("update_status", [f"Found {len(discovered)} devices", COLORS["primary"]]))
                self.ui_queue.put(("update_info", f"Found {len(discovered)} BLE device(s)"))
            else:
                self.ui_queue.put(("update_status", ["No devices found", COLORS["warning"]]))
                self.ui_queue.put(("update_info", "No BLE devices found"))

        except Exception as e:
            self.ui_queue.put(("show_error", f"Scan failed: {e}"))
            self.log(f"Scan error: {e}", tag="error")
        finally:
            self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.stop_btn, "disabled"]))
            self.scanning = False

    def stop_scan(self):
        self.stop_scan_event.set()
        self.scanning = False
        self.ui_queue.put(("update_status", ["Scan stopped", COLORS["danger"]]))
        self.ui_queue.put(("update_info", "Scan stopped manually"))
        self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
        self.ui_queue.put(("enable_button", [self.stop_btn, "disabled"]))
        self.log("Scan stopped", tag="warning")

    def on_device_select(self, event):
        selection = self.device_listbox.curselection()
        if selection:
            self.connect_button.config(state='normal')

    def connect_selected(self):
        selection = self.device_listbox.curselection()
        if not selection:
            return
        index = selection[0]
        if index < len(self.devices):
            device = self.devices[index]
            self.device_address = device.address

            self.ui_queue.put(("update_status", ["Connecting...", COLORS["warning"]]))
            self.ui_queue.put(("update_info", f"Connecting to {device.address}"))
            self.ui_queue.put(("enable_button", [self.connect_button, "disabled"]))
            self.ui_queue.put(("enable_button", [self.scan_button, "disabled"]))
            self.ui_queue.put(("enable_button", [self.stop_btn, "disabled"]))

            self.log(f"Attempting connection to {device.address}", tag="info")

            connect_thread = threading.Thread(target=self.connect_device_thread,
                                            args=(device.address, device.name or device.address), daemon=True)
            connect_thread.start()

    def connect_device_thread(self, address, name):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def connect():
                client = BleakClient(address)
                await client.connect(timeout=10.0)
                return client

            self.client = loop.run_until_complete(connect())
            loop.close()

            self.connected = True
            self.connection_start_time = time.time()

            device_name = name or address
            self.ui_queue.put(("update_status", ["Connected!", COLORS["success"]]))
            self.ui_queue.put(("update_info", f"Connected to: {device_name}"))
            self.ui_queue.put(("enable_button", [self.send_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.disconnect_btn, "normal"]))
            self.ui_queue.put(("enable_button", [self.connect_button, "disabled"]))

            self.ui_queue.put(("update_stats", ("device_label", device_name)))

            self.log(f"Successfully connected to {address}", tag="success")
            self.root.after(0, self.enable_controls)

        except Exception as e:
            self.connected = False
            self.client = None
            self.ui_queue.put(("show_error", f"Connection failed: {e}"))
            self.ui_queue.put(("update_status", ["Connection failed", COLORS["danger"]]))
            self.ui_queue.put(("update_info", "Connection failed"))
            self.ui_queue.put(("enable_button", [self.connect_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
            self.log(f"Connection error: {e}", tag="error")

    def disconnect_device(self):
        if not self.connected or not self.client:
            return
        self.disable_controls()

        self.ui_queue.put(("update_status", ["Disconnecting...", COLORS["warning"]]))
        self.ui_queue.put(("update_info", "Disconnecting from device..."))
        self.ui_queue.put(("enable_button", [self.disconnect_btn, "disabled"]))

        disconnect_thread = threading.Thread(target=self.disconnect_device_thread, daemon=True)
        disconnect_thread.start()

    def disconnect_device_thread(self):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def disconnect():
                if self.client and self.client.is_connected:
                    await self.client.disconnect()

            loop.run_until_complete(disconnect())
            loop.close()

            self.connected = False
            self.client = None
            self.connection_start_time = None

            self.ui_queue.put(("update_status", ["Disconnected", COLORS["danger"]]))
            self.ui_queue.put(("update_info", "No device connected"))
            self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.connect_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.send_button, "disabled"]))

            self.ui_queue.put(("update_stats", ("device_label", "None")))
            self.ui_queue.put(("update_stats", ("time_label", "00:00:00")))

            self.log("Disconnected from device", tag="warning")

        except Exception as e:
            self.ui_queue.put(("show_error", f"Disconnection error: {e}"))
            self.log(f"Disconnection error: {e}", tag="error")
        finally:
            self.connected = False
            self.client = None
            self.root.after(0, self.disable_controls)
            # Reset mode highlight
            self.select_mode(0, "Idle", silent=True)

    def enable_controls(self):
        for btn in self.control_buttons.values():
            btn.config(state='normal')
        for entry in self.float_entries:
            entry.config(state='normal')
        self.send_button.config(state='normal')
        self.control_frame.config(fg=COLORS["primary_light"])
        self.log("Controls enabled", tag="success")

    def disable_controls(self):
        for btn in self.control_buttons.values():
            btn.config(state='disabled')
        for entry in self.float_entries:
            entry.config(state='disabled')
        self.send_button.config(state='disabled')
        self.control_frame.config(fg=COLORS["text_muted"])
        self.log("Controls disabled", tag="warning")

    def show_controls(self, disabled=False):
        for widget in self.control_frame.winfo_children():
            widget.destroy()
        self.control_buttons.clear()
        state = 'disabled' if disabled else 'normal'
        color = COLORS["text_muted"] if disabled else COLORS["primary"]

        control_grid = tk.Frame(self.control_frame, bg=COLORS["bg_card"])
        control_grid.pack(expand=True)

        controls = [
            ("▲", "Forward", 1, 0, 1),
            ("◀", "Left", 3, 1, 0),
            ("⏹", "Stop", 0, 1, 1),
            ("▶", "Right", 4, 1, 2),
            ("▼", "Backward", 2, 2, 1)
        ]

        for symbol, text, value, row, col in controls:
            btn = ModernButton(control_grid, text=f"{symbol}\n{text}",
                               bg_color=color,
                               command=lambda v=value, t=text: self.control_press(v, t),
                               font=('Segoe UI', 14, 'bold'), width=6, height=3,
                               state=state)
            btn.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
            self.control_buttons[text] = btn

        for i in range(3):
            control_grid.grid_rowconfigure(i, weight=1)
            control_grid.grid_columnconfigure(i, weight=1)

    def control_press(self, value, label_text):
        if not self.connected:
            return
        self.control_byte = value
        btn = self.control_buttons.get(label_text)
        if btn:
            original_bg = btn['bg']
            btn.config(bg=COLORS["warning"])
            self.root.after(300, lambda b=btn, c=original_bg: b.config(bg=c))
        self.send_packet()

    def get_float_values(self):
        try:
            for i, entry in enumerate(self.float_entries):
                value = entry.get().strip()
                self.float_values[i] = float(value) if value else 0.0
            return True
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid float value at position {i+1}: {e}")
            return False

    def set_all_zero(self):
        for entry in self.float_entries:
            entry.delete(0, tk.END)
            entry.insert(0, "0.0")
        self.log("All 16 floats set to 0.0", tag="info")

    def set_test_pattern(self):
        test_values = [1.0,2.0,3.0,4.0,0.5,1.5,2.5,3.5,-1.0,-2.0,-3.0,-4.0,0.0,1.0,0.0,1.0]
        for i, entry in enumerate(self.float_entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(test_values[i]))
        self.log("Test pattern loaded for 16 floats", tag="info")

    def set_motor_control(self):
        motor_values = [1.5,-1.5,1.0,-1.0,0.8,-0.8,0.5,-0.5,0.3,-0.3,0.2,-0.2,0.1,-0.1,0.0,0.0]
        for i, entry in enumerate(self.float_entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(motor_values[i]))
        self.log("Motor control values loaded", tag="info")

    def set_pid_values(self):
        pid_values = [1.0,0.1,0.01,0.0,2.0,0.2,0.02,0.0,0.5,0.05,0.005,0.0,1.5,0.15,0.015,0.0]
        for i, entry in enumerate(self.float_entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(pid_values[i]))
        self.log("PID values loaded", tag="info")

    def send_packet(self):
        if not self.connected or not self.client:
            messagebox.showwarning("Not Connected", "Connect to device first.")
            return
        if not self.get_float_values():
            return

        packet = self.build_packet()
        self.log(f"Building {len(packet)}-byte packet", tag="info")
        self.ui_queue.put(("packet_sent", ["Building packet...", COLORS["warning"]]))

        send_thread = threading.Thread(target=self.send_packet_thread, args=(packet,), daemon=True)
        send_thread.start()

    def send_packet_thread(self, packet):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def send():
                await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False)

            loop.run_until_complete(send())
            loop.close()

            self.packets_sent += 1
            self.bytes_sent += len(packet)
            self.bytes_sent_label.config(text=f"Bytes sent: {self.bytes_sent}")

            self.ui_queue.put(("update_stats", ("packets_label", str(self.packets_sent))))
            self.ui_queue.put(("update_stats", ("bytes_label", f"{self.bytes_sent} B")))

            float_str = ", ".join([f"{f:.2f}" for f in self.float_values[:4]]) + "..."
            self.log(f"✓ {len(packet)}-byte packet sent! Control: {self.control_byte}", tag="success")
            self.log(f" First 4 floats: [{float_str}]", tag="info")

            self.ui_queue.put(("packet_sent", [f"✓ {len(packet)}-byte packet sent", COLORS["success"]]))

        except Exception as e:
            self.ui_queue.put(("show_error", f"Send failed: {e}"))
            self.log(f"✗ Send error: {e}", tag="error")

    def build_packet(self):
        packet = bytearray([0xA5, self.control_byte])
        for f in self.float_values:
            packet.extend(struct.pack('<f', f))
        packet.extend([0xAA, 0x5A])
        return packet

    def on_closing(self):
        if self.client and self.connected:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.client.disconnect())
                loop.close()
                self.log("Disconnected on exit", tag="info")
            except Exception as e:
                self.log(f"Error during disconnect: {e}", tag="error")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = RobotController()
    app.run()