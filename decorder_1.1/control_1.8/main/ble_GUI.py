import os
os.environ['BLEAK_LOGGING'] = '1' # Enable Bleak logs for debug
import tkinter as tk
from tkinter import ttk, messagebox, font
import struct
import threading
import asyncio
from bleak import BleakScanner, BleakClient
import sys
import queue
import time
try:
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
# Fix for Windows BLE scanning (MTA threading model)
if sys.platform == "win32":
    sys.coinit_flags = 0 # Force Multi-Threaded Apartment before imports
# BLE UUIDs (same as your Flutter app)
SERVICE_UUID = "cb341ded-25f3-244f-11ac-7ebc1a247a86"
RX_CHAR_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"
TX_CHAR_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # For telemetry (notifications)

# Motor names for display (index order: FLH, BLK, BLH, FLK, FRH, FRK, BRH, BRK)
MOTOR_NAMES = ["FLH", "BLK", "BLH", "FLK", "FRH", "FRK", "BRH", "BRK"]
# Leg masks: bit0=FLH, bit1=BLK, bit2=BLH, bit3=FLK, bit4=FRH, bit5=FRK, bit6=BRH, bit7=BRK
LEG_MASKS = {"All": 0xFF, "FL": 0x09, "FR": 0x30, "BL": 0x06, "BR": 0xC0}
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
        # Default Kp, Ki, Kd, F4, F5
        self.float_values = [0.25, 1.0, 0.002, 0.0, 0.0]
        self.control_byte = 0
        self.control_buttons = {}
        self.mode_buttons = {} # for mode selection
        self.orient_buttons = {} # for orientation
        self.posture_buttons = {} # for body posture
        self.devices = []
        self.ui_queue = queue.Queue()
        self.scanning = False
        self.stop_scan_event = threading.Event()
        self.float_entries = [None] * 5
        self.bytes_sent = 0
        self.packets_sent = 0
        self.connection_start_time = None
        self.current_base = 0
        self.gait = None
        self.current_orient = 9   # LEG_ORIENTATION_NORMAL
        self.current_posture = 30 # BODY_POSTURE_NORMAL
 
        # Add these lines:
        self.keep_alive_interval = 1.0 # Send keep-alive every 1 second
        self.keep_alive_running = False
        self.keep_alive_thread = None
        self.last_sent_packet = None # NEW: Store last manually sent packet
        self.last_sent_control_byte = 0 # NEW: Store last control byte
        self.last_sent_float_values = [0.0] * 5 # NEW: Store last float values
        # Telemetry
        self.telemetry_enabled = False
        self.telemetry_leg_mask = 0xFF
        self.telemetry_data = {name: {"target": 0.0, "encoder": 0.0} for name in MOTOR_NAMES}
        self.telemetry_log = []  # List of dicts for CSV export
        self.telemetry_logging = False  # When True, append to telemetry_log
        self.plot_history = []  # [(t, {motor: {target, encoder}}), ...] max 300 points
        self.plot_history_max = 300
        self.plot_t0 = None  # First timestamp for relative time axis
        self.telemetry_rx_count = 0  # Packets received (for status)
        self.ble_loop = None  # BLE event loop (must stay running for notifications)
        self.ble_thread = None  # Thread running the BLE loop
      
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
        self.setup_telemetry_tab()
        self.setup_log_tab()
        # Status bar
        status_bar = tk.Frame(self.root, bg=COLORS["bg_card"], height=30)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)
        status_bar.pack_propagate(False)
        self.packet_info = tk.Label(status_bar,
                                   text="Ready | Packet: 24 bytes (1B + 1B + 20B + 2B)",
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
        # Scrollable canvas for Controls tab
        canvas = tk.Canvas(controls_tab, bg=COLORS["bg_medium"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(controls_tab, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=COLORS["bg_medium"])
        window_id = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(window_id, width=e.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        # Mouse wheel scroll (Windows/Mac use delta, Linux uses Button-4/5)
        def _on_mousewheel(event):
            if hasattr(event, "delta"):
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
            elif event.num == 4:
                canvas.yview_scroll(-3, "units")
            elif event.num == 5:
                canvas.yview_scroll(3, "units")
        scrollable_frame.bind("<Enter>", lambda e: canvas.bind_all("<MouseWheel>", _on_mousewheel))
        scrollable_frame.bind("<Leave>", lambda e: canvas.unbind_all("<MouseWheel>"))
        for evt in ("<Button-4>", "<Button-5>"):
            scrollable_frame.bind(evt, _on_mousewheel)
        # Use scrollable_frame as parent for content
        top_frame = tk.Frame(scrollable_frame, bg=COLORS["bg_medium"])
        top_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        float_frame = tk.LabelFrame(top_frame, text=" Float Parameters (5 values) ",
                                   font=self.fonts['heading'], bg=COLORS["bg_card"],
                                   fg=COLORS["accent"], relief=tk.FLAT,
                                   borderwidth=2, padx=20, pady=20)
        float_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        # Default Kp, Ki, Kd for F1, F2, F3
        default_floats = ["0.25", "1.0", "0.002", "0.0", "0.0"]
        for i in range(5):
            row = i // 3
            col = i % 3
            frame = tk.Frame(float_frame, bg=COLORS["bg_card"])
            frame.grid(row=row, column=col, padx=10, pady=8, sticky='w')
            label_bg = self.get_gradient_color(i, 5)
            tk.Label(frame, text=f"F{i+1:02d}:", font=self.fonts['body'],
                     fg=COLORS["text_primary"], bg=label_bg,
                     padx=8, pady=3, relief=tk.RAISED,
                     borderwidth=1).pack(side=tk.LEFT)
            entry = tk.Entry(frame, bg=COLORS["bg_light"], fg=COLORS["text_primary"],
                             font=self.fonts['body'], width=12,
                             insertbackground=COLORS["primary_light"],
                             relief=tk.FLAT, borderwidth=1)
            entry.insert(0, default_floats[i])
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<FocusIn>", lambda e, w=entry: w.config(bg=COLORS["bg_medium"]))
            entry.bind("<FocusOut>", lambda e, w=entry: w.config(bg=COLORS["bg_light"]))
            value_label = tk.Label(frame, text=f"{float(default_floats[i]):.2f}", font=self.fonts['small'],
                                  fg=COLORS["text_muted"], bg=COLORS["bg_card"], width=6)
            value_label.pack(side=tk.LEFT, padx=5)
            entry.value_label = value_label
            entry.bind("<KeyRelease>", lambda e, idx=i: self.update_value_label(idx))
            self.float_entries[i] = entry
        bottom_frame = tk.Frame(scrollable_frame, bg=COLORS["bg_medium"])
        bottom_frame.pack(fill=tk.X, padx=10, pady=10)
                # Left side: Movement + Mode (using grid for reliable vertical stacking)
        left_container = tk.Frame(bottom_frame, bg=COLORS["bg_medium"])
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        # Configure grid weights
        left_container.grid_rowconfigure(0, weight=1)
        left_container.grid_rowconfigure(1, weight=0)
        left_container.grid_rowconfigure(2, weight=0)
        left_container.grid_rowconfigure(3, weight=0)
        left_container.grid_rowconfigure(4, weight=0)
        left_container.grid_columnconfigure(0, weight=1)
        # Robot Movement controls
        control_buttons_frame = tk.LabelFrame(left_container,
                                             text=" Robot Direction ",
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
                                   text=" Robot Gait Mode ",
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
            ("Homing", 0, COLORS["warning"]),
            ("Standby", 1, COLORS["primary"]),
            ("Idle", 2, COLORS["text_muted"]),
            ("Crawl", 4, COLORS["secondary"]),
            ("Trot", 5, COLORS["warning"]),
            ("Creep", 6, COLORS["accent"]),
            ("Walk", 7, COLORS["success"]),
            ("Gallop", 8, COLORS["danger"])
        ]
        self.mode_buttons = {}
        for i, (name, value, base_color) in enumerate(modes):
            btn = ModernButton(mode_grid,
                               text=name,
                               bg_color=base_color,
                               command=lambda v=value, n=name: self.select_mode(v, n),
                               width=12,
                               pady=8)
            btn.grid(row=i // 4, column=i % 4, padx=8, pady=6, sticky='ew')
            self.mode_buttons[value] = btn
        for c in range(4):
              mode_grid.grid_columnconfigure(c, weight=1)
        # Default to Homing
        self.select_mode(0, "Homing", silent=True)
        # Turning Gait Commands (for each gait: Straight, Turn L, Turn R)
        turn_frame = tk.LabelFrame(left_container,
                                   text=" Turning Gait Commands ",
                                   font=self.fonts['heading'],
                                   bg=COLORS["bg_card"],
                                   fg=COLORS["warning"],
                                   relief=tk.FLAT,
                                   borderwidth=2,
                                   padx=20,
                                   pady=20)
        turn_frame.grid(row=2, column=0, sticky='ew', padx=5, pady=(0, 10))
        # Trot: Straight(5), Left(11), Right(12)
        trot_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        trot_row.pack(fill=tk.X, pady=4)
        tk.Label(trot_row, text="Trot:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        for label, byte_val in [("Straight", 5), ("Turn L", 11), ("Turn R", 12)]:
            ModernButton(trot_row, text=label, bg_color=COLORS["warning"],
                        command=lambda b=byte_val: self.send_turn_cmd(b),
                        width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Creep: Straight(6), Left(13), Right(14)
        creep_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        creep_row.pack(fill=tk.X, pady=4)
        tk.Label(creep_row, text="Creep:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        for label, byte_val in [("Straight", 6), ("Turn L", 13), ("Turn R", 14)]:
            ModernButton(creep_row, text=label, bg_color=COLORS["accent"],
                        command=lambda b=byte_val: self.send_turn_cmd(b),
                        width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Crawl: Straight(4), Left(15), Right(16)
        crawl_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        crawl_row.pack(fill=tk.X, pady=4)
        tk.Label(crawl_row, text="Crawl:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        for label, byte_val in [("Straight", 4), ("Turn L", 15), ("Turn R", 16)]:
            ModernButton(crawl_row, text=label, bg_color=COLORS["secondary"],
                        command=lambda b=byte_val: self.send_turn_cmd(b),
                        width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Walk: Straight(7), Left(17), Right(18)
        walk_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        walk_row.pack(fill=tk.X, pady=4)
        tk.Label(walk_row, text="Walk:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        for label, byte_val in [("Straight", 7), ("Turn L", 17), ("Turn R", 18)]:
            ModernButton(walk_row, text=label, bg_color=COLORS["success"],
                        command=lambda b=byte_val: self.send_turn_cmd(b),
                        width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Gallop: Straight(8), Left(19), Right(20)
        gallop_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        gallop_row.pack(fill=tk.X, pady=4)
        tk.Label(gallop_row, text="Gallop:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        for label, byte_val in [("Straight", 8), ("Turn L", 19), ("Turn R", 20)]:
            ModernButton(gallop_row, text=label, bg_color=COLORS["danger"],
                        command=lambda b=byte_val: self.send_turn_cmd(b),
                        width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Pivot Turn: Trot CW(25)/CCW(26), Crawl CW(27)/CCW(28)
        pivot_row = tk.Frame(turn_frame, bg=COLORS["bg_card"])
        pivot_row.pack(fill=tk.X, pady=4)
        tk.Label(pivot_row, text="Pivot:", font=self.fonts['subheading'],
                 fg=COLORS["text_primary"], bg=COLORS["bg_card"], width=6,
                 anchor='w').pack(side=tk.LEFT, padx=(0, 5))
        ModernButton(pivot_row, text="Trot CW", bg_color=COLORS["warning"],
                     command=lambda: self.send_turn_cmd(25),
                     width=8, pady=4).pack(side=tk.LEFT, padx=3)
        ModernButton(pivot_row, text="Trot CCW", bg_color=COLORS["warning"],
                     command=lambda: self.send_turn_cmd(26),
                     width=8, pady=4).pack(side=tk.LEFT, padx=3)
        ModernButton(pivot_row, text="Crawl CW", bg_color=COLORS["secondary"],
                     command=lambda: self.send_turn_cmd(27),
                     width=8, pady=4).pack(side=tk.LEFT, padx=3)
        ModernButton(pivot_row, text="Crawl CCW", bg_color=COLORS["secondary"],
                     command=lambda: self.send_turn_cmd(28),
                     width=8, pady=4).pack(side=tk.LEFT, padx=3)
        # Leg Orientation selection
        orient_frame = tk.LabelFrame(left_container,
                                     text=" Leg Orientation ",
                                     font=self.fonts['heading'],
                                     bg=COLORS["bg_card"],
                                     fg=COLORS["accent"],
                                     relief=tk.FLAT,
                                     borderwidth=2,
                                     padx=20,
                                     pady=20)
        orient_frame.grid(row=3, column=0, sticky='ew', padx=5, pady=(0, 10))
        orient_grid = tk.Frame(orient_frame, bg=COLORS["bg_card"])
        orient_grid.pack(padx=10, pady=10)
        orients = [
            ("Normal", 9, COLORS["primary"]),
            ("Inverted", 10, COLORS["danger"])
        ]
        self.orient_buttons = {}
        for i, (name, value, base_color) in enumerate(orients):
            btn = ModernButton(orient_grid,
                               text=name,
                               bg_color=base_color,
                               command=lambda v=value, n=name: self.select_orient(v, n),
                               width=12,
                               pady=8)
            btn.grid(row=0, column=i, padx=8, pady=6, sticky='ew')
            self.orient_buttons[value] = btn
        for c in range(2):
            orient_grid.grid_columnconfigure(c, weight=1)
        # Default to Normal
        self.select_orient(9, "Normal", silent=True)
        # Body Posture selection
        posture_frame = tk.LabelFrame(left_container,
                                      text=" Body Posture ",
                                      font=self.fonts['heading'],
                                      bg=COLORS["bg_card"],
                                      fg=COLORS["warning"],
                                      relief=tk.FLAT,
                                      borderwidth=2,
                                      padx=20,
                                      pady=20)
        posture_frame.grid(row=4, column=0, sticky='ew', padx=5, pady=(0, 10))
        posture_grid = tk.Frame(posture_frame, bg=COLORS["bg_card"])
        posture_grid.pack(padx=10, pady=10)
        postures = [
            ("Normal", 30, COLORS["success"]),
            ("Low", 31, COLORS["warning"]),
            ("Crouch", 32, COLORS["danger"])
        ]
        self.posture_buttons = {}
        for i, (name, value, base_color) in enumerate(postures):
            btn = ModernButton(posture_grid,
                               text=name,
                               bg_color=base_color,
                               command=lambda v=value, n=name: self.select_posture(v, n),
                               width=12,
                               pady=8)
            btn.grid(row=0, column=i, padx=8, pady=6, sticky='ew')
            self.posture_buttons[value] = btn
        for c in range(3):
            posture_grid.grid_columnconfigure(c, weight=1)
        # Default to Normal
        self.select_posture(30, "Normal", silent=True)
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
    def setup_telemetry_tab(self):
        """Telemetry tab: leg selection, live display, export for Origin"""
        telemetry_tab = tk.Frame(self.notebook, bg=COLORS["bg_medium"])
        self.notebook.add(telemetry_tab, text="📊 Telemetry")
        # Leg selection
        leg_frame = tk.LabelFrame(telemetry_tab, text=" Leg Selection ",
                                  font=self.fonts['heading'], bg=COLORS["bg_card"],
                                  fg=COLORS["primary_light"], relief=tk.FLAT,
                                  borderwidth=2, padx=20, pady=20)
        leg_frame.pack(fill=tk.X, padx=10, pady=10)
        leg_grid = tk.Frame(leg_frame, bg=COLORS["bg_card"])
        leg_grid.pack()
        self.leg_vars = {}
        for i, (label, mask) in enumerate(LEG_MASKS.items()):
            var = tk.BooleanVar(value=(label == "All"))
            self.leg_vars[label] = (var, mask)
            cb = tk.Checkbutton(leg_grid, text=label, variable=var,
                                font=self.fonts['body'], fg=COLORS["text_primary"],
                                bg=COLORS["bg_card"], selectcolor=COLORS["bg_light"],
                                activebackground=COLORS["bg_card"],
                                command=self.update_telemetry_config)
            cb.grid(row=0, column=i, padx=15, pady=5, sticky='w')
        # Enable/Disable + Log toggle + Status
        ctrl_row = tk.Frame(leg_frame, bg=COLORS["bg_card"])
        ctrl_row.pack(pady=10)
        ModernButton(ctrl_row, text="Enable Telemetry", bg_color=COLORS["success"],
                     command=self.enable_telemetry, width=14).pack(side=tk.LEFT, padx=5)
        ModernButton(ctrl_row, text="Disable Telemetry", bg_color=COLORS["danger"],
                     command=self.disable_telemetry, width=14).pack(side=tk.LEFT, padx=5)
        ModernButton(ctrl_row, text="Re-subscribe TX", bg_color=COLORS["primary"],
                     command=self.resubscribe_telemetry, width=12).pack(side=tk.LEFT, padx=5)
        self.telemetry_status_label = tk.Label(ctrl_row, text="Status: Idle", font=self.fonts['body'],
                                               fg=COLORS["text_muted"], bg=COLORS["bg_card"])
        self.telemetry_status_label.pack(side=tk.LEFT, padx=15)
        self.log_telemetry_var = tk.BooleanVar(value=False)
        tk.Checkbutton(ctrl_row, text="Log for export", variable=self.log_telemetry_var,
                       font=self.fonts['body'], fg=COLORS["text_primary"],
                       bg=COLORS["bg_card"],
                       command=lambda: setattr(self, 'telemetry_logging', self.log_telemetry_var.get())).pack(side=tk.LEFT, padx=15)
        # Live display
        disp_frame = tk.LabelFrame(telemetry_tab, text=" Live Motor Data ",
                                  font=self.fonts['heading'], bg=COLORS["bg_card"],
                                  fg=COLORS["accent"], relief=tk.FLAT,
                                  borderwidth=2, padx=20, pady=20)
        disp_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.telemetry_labels = {}
        for i, name in enumerate(MOTOR_NAMES):
            row = tk.Frame(disp_frame, bg=COLORS["bg_card"])
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=f"{name}:", font=self.fonts['mono'], width=5,
                     fg=COLORS["text_secondary"], bg=COLORS["bg_card"]).pack(side=tk.LEFT)
            lbl_target = tk.Label(row, text="Target: 0.0", font=self.fonts['mono'],
                                 fg=COLORS["primary_light"], bg=COLORS["bg_card"], width=18)
            lbl_target.pack(side=tk.LEFT, padx=5)
            lbl_enc = tk.Label(row, text="Encoder: 0", font=self.fonts['mono'],
                               fg=COLORS["secondary"], bg=COLORS["bg_card"], width=18)
            lbl_enc.pack(side=tk.LEFT, padx=5)
            self.telemetry_labels[name] = (lbl_target, lbl_enc)
        # Plot: Target vs Encoder on same graph
        if MATPLOTLIB_AVAILABLE:
            plot_frame = tk.LabelFrame(telemetry_tab, text=" Target vs Encoder Plot ",
                                       font=self.fonts['heading'], bg=COLORS["bg_card"],
                                       fg=COLORS["primary_light"], relief=tk.FLAT,
                                       borderwidth=2, padx=20, pady=20)
            plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            plot_ctrl = tk.Frame(plot_frame, bg=COLORS["bg_card"])
            plot_ctrl.pack(fill=tk.X, pady=(0, 5))
            tk.Label(plot_ctrl, text="Motor:", font=self.fonts['body'],
                     fg=COLORS["text_secondary"], bg=COLORS["bg_card"]).pack(side=tk.LEFT, padx=(0, 5))
            self.plot_motor_var = tk.StringVar(value=MOTOR_NAMES[0])
            motor_combo = ttk.Combobox(plot_ctrl, textvariable=self.plot_motor_var,
                                       values=MOTOR_NAMES, state="readonly", width=8)
            motor_combo.pack(side=tk.LEFT, padx=5)
            motor_combo.bind("<<ComboboxSelected>>", lambda e: self.refresh_telemetry_plot())
            ModernButton(plot_ctrl, text="Clear Plot", bg_color=COLORS["text_muted"],
                         command=self.clear_telemetry_plot, width=10).pack(side=tk.LEFT, padx=10)
            self.fig = Figure(figsize=(10, 5), dpi=100, facecolor=COLORS["bg_card"])
            self.ax = self.fig.add_subplot(111)
            self.ax.set_facecolor(COLORS["bg_light"])
            self.ax.tick_params(colors=COLORS["text_primary"])
            self.ax.xaxis.label.set_color(COLORS["text_primary"])
            self.ax.yaxis.label.set_color(COLORS["text_primary"])
            self.ax.spines['bottom'].set_color(COLORS["border"])
            self.ax.spines['top'].set_color(COLORS["border"])
            self.ax.spines['left'].set_color(COLORS["border"])
            self.ax.spines['right'].set_color(COLORS["border"])
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Position")
            self.plot_canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
            self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.plot_canvas.draw()
        else:
            tk.Label(telemetry_tab, text="Install matplotlib for plotting: pip install matplotlib",
                     font=self.fonts['body'], fg=COLORS["warning"], bg=COLORS["bg_medium"]).pack(pady=10)
        # Export
        export_row = tk.Frame(telemetry_tab, bg=COLORS["bg_medium"])
        export_row.pack(fill=tk.X, padx=10, pady=10)
        ModernButton(export_row, text="Export to CSV (Origin)", bg_color=COLORS["accent"],
                     command=self.export_telemetry_csv).pack(side=tk.LEFT, padx=5)
        ModernButton(export_row, text="Clear Log", bg_color=COLORS["text_muted"],
                     command=self.clear_telemetry_log).pack(side=tk.LEFT, padx=5)

    def update_telemetry_config(self):
        """Compute leg mask from checkboxes and send to ESP32"""
        mask = 0
        for label, (var, m) in self.leg_vars.items():
            if var.get():
                mask |= m
        self.telemetry_leg_mask = mask if mask else 0xFF  # 0 = use all when enabling
        if self.connected and self.client:
            self.send_telemetry_config()

    def send_telemetry_config(self):
        """Send [0xA6, leg_mask] to ESP32"""
        if not self.connected or not self.client:
            return
        packet = bytes([0xA6, self.telemetry_leg_mask])
        threading.Thread(target=self._send_telemetry_config_thread, args=(packet,), daemon=True).start()

    def _send_telemetry_config_thread(self, packet):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False))
            loop.close()
        except Exception as e:
            self.ui_queue.put(("show_error", f"Telemetry config failed: {e}"))

    def enable_telemetry(self):
        if not self.connected or not self.client:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        self.update_telemetry_config()
        self.telemetry_enabled = True
        if self.telemetry_leg_mask == 0:
            self.telemetry_leg_mask = 0xFF  # default to all
        self.send_telemetry_config()
        self.telemetry_rx_count = 0
        if hasattr(self, 'telemetry_status_label'):
            self.telemetry_status_label.config(text="Status: Enabled, waiting for data...", fg=COLORS["warning"])
        self.log("Telemetry enabled (sent config mask=0x%02X)" % self.telemetry_leg_mask, tag="success")

    def resubscribe_telemetry(self):
        """Re-subscribe to TX notifications (use if no data appears)"""
        if not self.connected or not self.client:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        def _do_subscribe():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.client.start_notify(TX_CHAR_UUID, self.on_telemetry_notification))
                loop.close()
                self.ui_queue.put(("log", ("[%s] TX notifications re-subscribed" % time.strftime("%H:%M:%S"), "info")))
            except Exception as e:
                self.ui_queue.put(("show_error", f"Re-subscribe failed: {e}"))
        threading.Thread(target=_do_subscribe, daemon=True).start()
        self.log("Re-subscribing to TX notifications...", tag="info")

    def disable_telemetry(self):
        packet = bytes([0xA6, 0])  # leg_mask=0 disables
        if self.connected and self.client:
            threading.Thread(target=self._send_telemetry_config_thread, args=(packet,), daemon=True).start()
        self.telemetry_enabled = False
        self.plot_t0 = None
        self.plot_history.clear()
        if hasattr(self, 'telemetry_status_label'):
            self.telemetry_status_label.config(text="Status: Idle", fg=COLORS["text_muted"])
        if MATPLOTLIB_AVAILABLE and hasattr(self, 'refresh_telemetry_plot'):
            self.root.after(0, self.refresh_telemetry_plot)  # Clear plot on next tick
        self.log("Telemetry disabled", tag="warning")

    def on_telemetry_notification(self, sender, data):
        """Parse [0xAB, leg_mask, target0, enc0, ...] and update display (called from BLE thread)"""
        try:
            if data is None or len(data) < 3 or data[0] != 0xAB:
                return
            leg_mask = data[1]
            offset = 2
            new_data = {}
            for i, name in enumerate(MOTOR_NAMES):
                if not (leg_mask & (1 << i)):
                    continue
                if offset + 8 > len(data):
                    break
                target = struct.unpack('<f', data[offset:offset+4])[0]
                enc = struct.unpack('<f', data[offset+4:offset+8])[0]
                offset += 8
                new_data[name] = {"target": target, "encoder": enc}
            self.telemetry_data.update(new_data)
            if self.telemetry_logging:
                row = {"time": time.time()}
                for n, d in self.telemetry_data.items():
                    row[f"{n}_target"] = d["target"]
                    row[f"{n}_encoder"] = d["encoder"]
                self.telemetry_log.append(row)
            self.telemetry_rx_count += 1
            self.ui_queue.put(("update_telemetry", dict(self.telemetry_data)))
        except Exception as e:
            self.ui_queue.put(("log", ("[%s] Telemetry parse error: %s" % (time.strftime("%H:%M:%S"), str(e)), "error")))

    def export_telemetry_csv(self):
        """Export telemetry_log to CSV for Origin"""
        if not self.telemetry_log:
            messagebox.showinfo("Export", "No telemetry data logged. Enable 'Log for export' and run telemetry.")
            return
        try:
            filename = f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w') as f:
                if self.telemetry_log:
                    headers = list(self.telemetry_log[0].keys())
                    f.write(",".join(headers) + "\n")
                    for row in self.telemetry_log:
                        f.write(",".join(str(row.get(h, "")) for h in headers) + "\n")
            self.log(f"Exported to {filename}", tag="success")
            messagebox.showinfo("Export", f"Saved to {filename}")
        except Exception as e:
            self.log(f"Export failed: {e}", tag="error")
            messagebox.showerror("Export Error", str(e))

    def clear_telemetry_log(self):
        self.telemetry_log.clear()
        self.log("Telemetry log cleared", tag="info")

    def clear_telemetry_plot(self):
        """Clear plot history without disabling telemetry"""
        self.plot_history.clear()
        self.plot_t0 = None
        if MATPLOTLIB_AVAILABLE:
            self.refresh_telemetry_plot()
        self.log("Plot cleared", tag="info")

    def refresh_telemetry_plot(self):
        """Redraw plot with target and encoder for selected motor"""
        if not MATPLOTLIB_AVAILABLE or not hasattr(self, 'ax'):
            return
        motor = self.plot_motor_var.get() if hasattr(self, 'plot_motor_var') else MOTOR_NAMES[0]
        times = []
        targets = []
        encoders = []
        for t, d in self.plot_history:
            if motor in d:
                times.append(t - self.plot_t0)
                targets.append(d[motor]["target"])
                encoders.append(d[motor]["encoder"])
        self.ax.clear()
        self.ax.set_facecolor(COLORS["bg_light"])
        self.ax.tick_params(colors=COLORS["text_primary"])
        self.ax.xaxis.label.set_color(COLORS["text_primary"])
        self.ax.yaxis.label.set_color(COLORS["text_primary"])
        self.ax.spines['bottom'].set_color(COLORS["border"])
        self.ax.spines['top'].set_color(COLORS["border"])
        self.ax.spines['left'].set_color(COLORS["border"])
        self.ax.spines['right'].set_color(COLORS["border"])
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Position")
        if times:
            self.ax.plot(times, targets, color=COLORS["primary_light"], label=f"{motor} Target", linewidth=1.5)
            self.ax.plot(times, encoders, color=COLORS["secondary"], label=f"{motor} Encoder", linewidth=1.5)
            self.ax.legend(loc='upper right', facecolor=COLORS["bg_card"], labelcolor=COLORS["text_primary"])
        self.plot_canvas.draw_idle()

    def send_turn_cmd(self, control_byte_val):
        """Send a turning gait command directly (bypasses gait selection)."""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        self.control_byte = control_byte_val
        self.send_packet()

    def select_mode(self, mode_value, mode_name, silent=False):
        """Set robot mode and update button highlight"""
        if not self.connected and not silent:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        self.control_byte = mode_value
        self.current_base = mode_value
        self.gait = {4: 'crawl', 5: 'trot', 6: 'creep', 7: 'walk', 8: 'gallop'}.get(mode_value, None)
        # Highlight active button
        for val, btn in self.mode_buttons.items():
            if val == mode_value:
                btn.config(bg=COLORS["success"], fg=COLORS["bg_dark"])
            else:
                colors = {0: COLORS["warning"], 1: COLORS["primary"], 2: COLORS["text_muted"],
                          4: COLORS["secondary"], 5: COLORS["warning"], 6: COLORS["accent"],
                          7: COLORS["success"], 8: COLORS["danger"]}
                btn.config(bg=colors.get(val, COLORS["primary"]), fg=COLORS["text_primary"])
        if not silent:
            self.log(f"Mode changed to: {mode_name} ({mode_value})", tag="info")
            self.send_packet()
    def select_orient(self, orient_value, orient_name, silent=False):
        """Set leg orientation and update button highlight"""
        if not self.connected and not silent:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        self.control_byte = orient_value
        self.current_orient = orient_value
        # Highlight active button
        for val, btn in self.orient_buttons.items():
            if val == orient_value:
                btn.config(bg=COLORS["success"], fg=COLORS["bg_dark"])
            else:
                colors = {9: COLORS["primary"], 10: COLORS["danger"]}
                btn.config(bg=colors[val], fg=COLORS["text_primary"])
        if not silent:
            self.log(f"Orientation changed to: {orient_name} ({orient_value})", tag="info")
            self.send_packet()
    def select_posture(self, posture_value, posture_name, silent=False):
        """Set body posture and update button highlight"""
        if not self.connected and not silent:
            messagebox.showwarning("Not Connected", "Connect to a device first.")
            return
        self.control_byte = posture_value
        self.current_posture = posture_value
        # Highlight active button
        for val, btn in self.posture_buttons.items():
            if val == posture_value:
                btn.config(bg=COLORS["success"], fg=COLORS["bg_dark"])
            else:
                colors = {30: COLORS["success"], 31: COLORS["warning"], 32: COLORS["danger"]}
                btn.config(bg=colors[val], fg=COLORS["text_primary"])
        if not silent:
            self.log(f"Posture changed to: {posture_name} ({posture_value})", tag="info")
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
                elif msg_type == "update_telemetry":
                    # Update telemetry labels, status, and plot on main thread
                    if hasattr(self, 'telemetry_status_label'):
                        self.telemetry_status_label.config(
                            text="Status: Receiving (%d packets)" % getattr(self, 'telemetry_rx_count', 0),
                            fg=COLORS["success"])
                    if hasattr(self, 'telemetry_labels') and data:
                        for name, d in data.items():
                            if name in self.telemetry_labels:
                                self.telemetry_labels[name][0].config(text=f"Target: {d['target']:.1f}")
                                self.telemetry_labels[name][1].config(text=f"Encoder: {d['encoder']:.0f}")
                        if MATPLOTLIB_AVAILABLE and hasattr(self, 'plot_history'):
                            t = time.time()
                            if self.plot_t0 is None:
                                self.plot_t0 = t
                            self.plot_history.append((t, dict(data)))
                            if len(self.plot_history) > self.plot_history_max:
                                self.plot_history.pop(0)
                            self.refresh_telemetry_plot()
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)
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
                # Use detection_callback to get RSSI in real-time
                devices_found = []
             
                def detection_callback(device, advertisement_data):
                    rssi = advertisement_data.rssi if advertisement_data else -100
                    devices_found.append((device, rssi))
             
                scanner = BleakScanner(detection_callback=detection_callback)
                await scanner.start()
                await asyncio.sleep(10.0) # Scan for 10 seconds
                await scanner.stop()
             
                # Remove duplicates and keep best RSSI
                unique_devices = {}
                for device, rssi in devices_found:
                    if device.address not in unique_devices or rssi > unique_devices[device.address][1]:
                        unique_devices[device.address] = (device, rssi)
             
                return [(device, rssi) for device, rssi in unique_devices.values()]
            discovered = loop.run_until_complete(scan())
            loop.close()
            device_strings = []
            # Sort by RSSI (strongest first)
            discovered.sort(key=lambda x: x[1], reverse=True)
         
            for device, rssi in discovered:
                name = device.name or "Unknown"
                display = f"{name[:25]:25} | {device.address} | RSSI: {rssi:3}"
                device_strings.append(display)
                self.devices.append(device)
                self.log(f"Found: {name} ({device.address}) RSSI: {rssi}", tag="info")
                # Warn about weak signals
                if rssi < -80:
                    self.log(f" ⚠ Weak signal for {name}", tag="warning")
            self.ui_queue.put(("update_list", device_strings))
            if discovered:
                count = len(discovered)
                status = f"Found {count} device(s)"
                self.ui_queue.put(("update_status", [status, COLORS["primary"]]))
                self.ui_queue.put(("update_info", f"Found {count} BLE device(s)"))
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
            self.ble_loop = loop
            self.ble_thread = threading.current_thread()

            async def connect():
                client = BleakClient(address,
                                    timeout=15.0,
                                    disconnected_callback=self.on_disconnected)
                await client.connect()
                await client.start_notify(TX_CHAR_UUID, self.on_telemetry_notification)
                return client

            self.client = loop.run_until_complete(connect())
            self.connected = True

            self.root.after(1000, self.update_connection_time)
            self.root.after(2000, self.monitor_signal_strength)

            if hasattr(self, 'keep_alive_running') and self.keep_alive_running:
                self.keep_alive_running = False
                if hasattr(self, 'keep_alive_thread') and self.keep_alive_thread.is_alive():
                    self.keep_alive_thread.join(timeout=1.0)

            self.keep_alive_running = True
            self.keep_alive_thread = threading.Thread(target=self.keep_alive_loop, daemon=True)
            self.keep_alive_thread.start()
            self.log("Keep-alive started (BLE heartbeat active)", tag="info")
            self.connection_start_time = time.time()
            device_name = name or address
            self.ui_queue.put(("update_status", ["Connected!", COLORS["success"]]))
            self.ui_queue.put(("update_info", f"Connected to: {device_name}"))
            self.ui_queue.put(("enable_button", [self.send_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.disconnect_btn, "normal"]))
            self.ui_queue.put(("enable_button", [self.connect_button, "disabled"]))
            self.ui_queue.put(("update_stats", ("device_label", device_name)))
            self.log(f"Successfully connected to {address}", tag="success")
            self.log("TX notifications enabled (telemetry ready)", tag="info")
            self.root.after(0, self.enable_controls)

            # Keep loop running - REQUIRED for BLE notifications to be delivered
            loop.run_forever()
        except Exception as e:
            self.connected = False
            self.client = None
            error_msg = str(e)
            if "timeout" in error_msg.lower():
                error_msg = "Connection timeout. Device might be out of range or busy."
            self.ui_queue.put(("show_error", f"Connection failed: {error_msg}"))
            self.ui_queue.put(("update_status", ["Connection failed", COLORS["danger"]]))
            self.ui_queue.put(("update_info", "Connection failed"))
            self.ui_queue.put(("enable_button", [self.connect_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
            self.log(f"Connection error: {error_msg}", tag="error")
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
            if self.ble_loop and self.client:
                async def disconnect_and_stop():
                    try:
                        if self.client and self.client.is_connected:
                            await self.client.disconnect()
                    finally:
                        self.ble_loop.stop()

                future = asyncio.run_coroutine_threadsafe(disconnect_and_stop(), self.ble_loop)
                future.result(timeout=10)
            if self.ble_thread and self.ble_thread.is_alive():
                self.ble_thread.join(timeout=3)
        except Exception as e:
            self.ui_queue.put(("log", ("[%s] Disconnect: %s" % (time.strftime("%H:%M:%S"), str(e)), "error")))
        finally:
            self.connected = False
            self.client = None
            self.ble_loop = None
            self.ble_thread = None
            self.connection_start_time = None
            self.ui_queue.put(("update_status", ["Disconnected", COLORS["danger"]]))
            self.ui_queue.put(("update_info", "No device connected"))
            self.ui_queue.put(("enable_button", [self.scan_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.connect_button, "normal"]))
            self.ui_queue.put(("enable_button", [self.send_button, "disabled"]))
            self.ui_queue.put(("update_stats", ("device_label", "None")))
            self.ui_queue.put(("update_stats", ("time_label", "00:00:00")))
            self.log("Disconnected from device", tag="warning")
            self.root.after(0, self.disable_controls)
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
            ("▲", "Forward", 'forward', 0, 1),
            ("◀", "Left", 'left', 1, 0),
            ("⏹", "Stop", 'stop', 1, 1),
            ("▶", "Right", 'right', 1, 2),
            ("▼", "Backward", 'backward', 2, 1)
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
    def control_press(self, direction, label_text):
        if not self.connected:
            return
        btn = self.control_buttons.get(label_text)
        if btn:
            original_bg = btn['bg']
            btn.config(bg=COLORS["warning"])
            self.root.after(300, lambda b=btn, c=original_bg: b.config(bg=c))
        if direction == 'stop':
            self.select_mode(2, "Idle")  # CPG_MODE_IDLE: motors stop immediately
        elif direction == 'forward':
            self.control_byte = self.current_base
            self.send_packet()
        elif direction == 'backward':
            messagebox.showwarning("Not Supported", "Backward movement not implemented.")
            return
        elif direction == 'left':
            if self.gait is None:
                return
            turn_map = {'crawl': 15, 'creep': 13, 'trot': 11, 'walk': 17, 'gallop': 19}  # MODE_*_LEFT
            self.control_byte = turn_map.get(self.gait, self.current_base)
            self.send_packet()
        elif direction == 'right':
            if self.gait is None:
                return
            turn_map = {'crawl': 16, 'creep': 14, 'trot': 12, 'walk': 18, 'gallop': 20}  # MODE_*_RIGHT
            self.control_byte = turn_map.get(self.gait, self.current_base)
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
        self.log("All 5 floats set to 0.0", tag="info")
    def set_test_pattern(self):
        test_values = [1.0,2.0,3.0,4.0,0.5]
        for i, entry in enumerate(self.float_entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(test_values[i]))
        self.log("Test pattern loaded for 5 floats", tag="info")
    def set_motor_control(self):
        motor_values = [1.5,-1.5,1.0,-1.0,0.8]
        for i, entry in enumerate(self.float_entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(motor_values[i]))
        self.log("Motor control values loaded", tag="info")
    def set_pid_values(self):
        pid_values = [0.25, 1.0, 0.002, 0.0, 2.0]  # Kp, Ki, Kd, F4, F5
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
                # Try up to 3 times with exponential backoff
                for attempt in range(3):
                    try:
                        await self.client.write_gatt_char(
                            RX_CHAR_UUID,
                            packet,
                            response=False
                        )
                        return True
                    except Exception as e:
                        if attempt == 2:
                            raise e
                        await asyncio.sleep(0.1 * (2 ** attempt)) # Exponential backoff
             
                return False
            success = loop.run_until_complete(send())
            loop.close()
            if success:
                # Store the last sent values for keep-alive
                self.last_sent_packet = packet
                self.last_sent_control_byte = self.control_byte
                self.last_sent_float_values = self.float_values.copy()
              
                self.packets_sent += 1
                self.bytes_sent += len(packet)
                self.bytes_sent_label.config(text=f"Bytes sent: {self.bytes_sent}")
                self.ui_queue.put(("update_stats", ("packets_label", str(self.packets_sent))))
                self.ui_queue.put(("update_stats", ("bytes_label", f"{self.bytes_sent} B")))
                float_str = ", ".join([f"{f:.2f}" for f in self.float_values])
                self.log(f"✓ {len(packet)}-byte packet sent! Control: {self.control_byte}", tag="success")
                self.log(f" Floats: [{float_str}]", tag="info")
                self.ui_queue.put(("packet_sent", [f"✓ {len(packet)}-byte packet sent", COLORS["success"]]))
            else:
                raise Exception("Failed after 3 attempts")
        except Exception as e:
            self.ui_queue.put(("show_error", f"Send failed: {e}"))
            self.log(f"✗ Send error: {e}", tag="error")
            # Check if connection is lost
            if "not connected" in str(e).lower() or "disconnected" in str(e).lower():
                self.ui_queue.put(("update_status", ["Connection lost", COLORS["danger"]]))
                self.disconnect_device()
    def update_connection_time(self):
        """Update the connection time display"""
        if self.connected and self.connection_start_time:
            elapsed = time.time() - self.connection_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            self.ui_queue.put(("update_stats", ("time_label", time_str)))
     
        # Schedule next update if connected
        if self.connected:
            self.root.after(1000, self.update_connection_time) # Update every second
    def build_packet(self):
        packet = bytearray([0xA5, self.control_byte])
        for f in self.float_values:
            packet.extend(struct.pack('<f', f))
        packet.extend([0xAA, 0x5A])
        return packet
    def on_closing(self):
        if self.client and self.connected and self.ble_loop:
            try:
                async def disco():
                    if self.client and self.client.is_connected:
                        await self.client.disconnect()
                asyncio.run_coroutine_threadsafe(disco(), self.ble_loop).result(timeout=5)
                self.ble_loop.call_soon_threadsafe(self.ble_loop.stop)
                if self.ble_thread and self.ble_thread.is_alive():
                    self.ble_thread.join(timeout=3)
                self.log("Disconnected on exit", tag="info")
            except Exception as e:
                self.log(f"Error during disconnect: {e}", tag="error")
        self.root.destroy()
    def keep_alive_loop(self):
        """Continuously send last manual packet to keep BLE connection alive"""
        while self.keep_alive_running:
            if self.connected and self.client and hasattr(self.client, 'is_connected') and self.client.is_connected:
                # Only send keep-alive if we have a manually sent packet to repeat
                if self.last_sent_packet:
                    keep_alive_packet = self.last_sent_packet
                else:
                    # If no manual packet yet, send idle state (only once)
                    keep_alive_packet = bytearray([0xA5, 0x02]) # Control byte 2 (Idle - motors stop)
                    keep_alive_packet.extend([0] * 20) # All zeros for floats
                    keep_alive_packet.extend([0xAA, 0x5A])
                    self.last_sent_packet = keep_alive_packet
              
                # Use a separate event loop for keep-alive
                try:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                 
                    async def send_keep_alive():
                        try:
                            await asyncio.wait_for(
                                self.client.write_gatt_char(RX_CHAR_UUID, keep_alive_packet, response=False),
                                timeout=2.0
                            )
                            return True
                        except asyncio.TimeoutError:
                            return False
                        except Exception as e:
                            return False
                 
                    success = loop.run_until_complete(send_keep_alive())
                    loop.close()
                 
                    if not success:
                        # Try to reconnect if keep-alive fails
                        if self.connected:
                            self.ui_queue.put(("update_status", ["Connection lost", COLORS["danger"]]))
                            self.disconnect_device()
             
                except Exception:
                    pass # Silently fail for keep-alive errors
            else:
                # Connection lost, exit keep-alive loop
                if self.keep_alive_running:
                    self.keep_alive_running = False
         
            # Sleep for interval
            time.sleep(self.keep_alive_interval)
    def on_disconnected(self, client):
        """Called when device disconnects unexpectedly"""
        if self.connected:
            self.log("Device disconnected unexpectedly", tag="error")
            self.ui_queue.put(("update_status", ["Disconnected", COLORS["danger"]]))
            self.ui_queue.put(("update_info", "Connection lost"))
            self.ui_queue.put(("enable_button", [self.send_button, "disabled"]))
            self.connected = False
            self.client = None
            if self.ble_loop and self.ble_loop.is_running():
                self.ble_loop.call_soon_threadsafe(self.ble_loop.stop)
            self.root.after(0, self.disable_controls)
    def monitor_signal_strength(self):
        """Periodically check signal strength and warn if weak"""
        if self.connected and self.client:
            try:
                # Get RSSI from connected device (async)
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                async def get_rssi_async():
                    return await self.client.get_rssi()
                rssi = loop.run_until_complete(get_rssi_async())
                loop.close()
                if rssi:
                    self.ui_queue.put(("update_stats", ("rssi_label", f"{rssi} dBm")))
                    if rssi < -80:
                        self.ui_queue.put(("update_info", f"Weak signal: {rssi} dBm"))
                        if rssi < -90:
                            self.log(f"Very weak signal: {rssi} dBm", tag="warning")
            except:
                pass
     
        # Schedule next check if still connected
        if self.connected:
            self.root.after(5000, self.monitor_signal_strength) # Check every 5 seconds
  
    def run(self):
        self.root.mainloop()
if __name__ == "__main__":
    app = RobotController()
    app.run()