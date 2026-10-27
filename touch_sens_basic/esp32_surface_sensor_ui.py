import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports

# ============================================================
# Configuration
# ============================================================

BAUD_RATE = 115200

# CSV column -> (x position, y position)
# Positions are percentages of the surface canvas.
# These match the relative sensor locations in the uploaded layout.
SENSORS = {
    0: (0.34, 0.29),  # left
    2: (0.50, 0.29),  # center
    4: (0.66, 0.29),  # right
    3: (0.50, 0.13),  # top
}

# If you have a sensor in column 1, add it here, for example:
# 1: (0.42, 0.29),

# ============================================================
# Serial setup
# ============================================================

ser = None

def connect_serial():
    global ser

    port = port_var.get()

    try:
        if ser is not None and ser.is_open:
            ser.close()

        ser = serial.Serial(port, BAUD_RATE, timeout=0)
        status_var.set(f"Connected: {port}")
        status_label.config(foreground="green")
    except Exception as e:
        ser = None
        status_var.set(f"Connection failed: {e}")
        status_label.config(foreground="red")


def read_serial():
    """Read any complete CSV lines currently available."""
    global ser

    if ser is not None and ser.is_open:
        try:
            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                try:
                    values = [float(x.strip()) for x in line.split(",")]
                    update_sensors(values)
                except ValueError:
                    # Ignore malformed/header lines
                    pass

        except serial.SerialException as e:
            status_var.set(f"Serial error: {e}")
            status_label.config(foreground="red")

    root.after(20, read_serial)


# ============================================================
# Surface drawing
# ============================================================

sensor_items = {}

def draw_surface():
    canvas.delete("surface")

    w = canvas.winfo_width()
    h = canvas.winfo_height()

    # Approximate outline of the uploaded surface.
    # Coordinates are relative so the drawing scales with the window.
    points = [
        (0.10*w, 0.27*h),
        (0.18*w, 0.18*h),
        (0.22*w, 0.08*h),
        (0.30*w, 0.03*h),
        (0.50*w, 0.01*h),
        (0.70*w, 0.03*h),
        (0.78*w, 0.08*h),
        (0.82*w, 0.18*h),
        (0.90*w, 0.27*h),
        (0.98*w, 0.36*h),
        (0.90*w, 0.45*h),
        (0.84*w, 0.50*h),
        (0.78*w, 0.92*h),
        (0.22*w, 0.92*h),
        (0.16*w, 0.50*h),
        (0.10*w, 0.45*h),
        (0.02*w, 0.36*h),
    ]

    flat_points = []
    for x, y in points:
        flat_points.extend([x, y])

    canvas.create_polygon(
        flat_points,
        outline="black",
        fill="white",
        width=3,
        tags="surface"
    )

    # Center opening / slot
    slot = [
        (0.50*w, 0.24*h),
        (0.47*w, 0.34*h),
        (0.47*w, 0.48*h),
        (0.50*w, 0.60*h),
        (0.53*w, 0.48*h),
        (0.53*w, 0.34*h),
    ]

    flat_slot = []
    for x, y in slot:
        flat_slot.extend([x, y])

    canvas.create_polygon(
        flat_slot,
        outline="black",
        fill="white",
        width=2,
        tags="surface"
    )


def draw_sensors():
    """Draw circles and their current values."""
    for item in sensor_items.values():
        canvas.delete(item[0])
        canvas.delete(item[1])

    sensor_items.clear()

    w = canvas.winfo_width()
    h = canvas.winfo_height()

    radius = min(w, h) * 0.045

    for column, (xp, yp) in SENSORS.items():
        x = xp * w
        y = yp * h

        circle = canvas.create_oval(
            x-radius, y-radius,
            x+radius, y+radius,
            fill="#a9dff7",
            outline="#2386b8",
            width=2,
            tags="sensor"
        )

        text = canvas.create_text(
            x, y,
            text="--",
            font=("Arial", max(10, int(radius * 0.55))),
            fill="black",
            tags="sensor"
        )

        sensor_items[column] = (circle, text)


def redraw():
    draw_surface()
    draw_sensors()


def update_sensors(values):
    """Update displayed values from a CSV row."""
    for column, (_, text_id) in sensor_items.items():
        if column < len(values):
            value = values[column]

            # Change this formatting if you need more/less precision.
            canvas.itemconfig(text_id, text=f"{value:.2f}")


# ============================================================
# Window
# ============================================================

root = tk.Tk()
root.title("ESP32 Surface Sensor Monitor")
root.geometry("900x700")
root.minsize(600, 500)

# Top controls
controls = ttk.Frame(root, padding=8)
controls.pack(fill="x")

ttk.Label(controls, text="Serial Port:").pack(side="left")

port_var = tk.StringVar()

ports = [p.device for p in serial.tools.list_ports.comports()]

port_box = ttk.Combobox(
    controls,
    textvariable=port_var,
    values=ports,
    width=12
)
port_box.pack(side="left", padx=5)

if ports:
    port_box.current(0)

ttk.Button(
    controls,
    text="Connect",
    command=connect_serial
).pack(side="left", padx=5)

status_var = tk.StringVar(value="Not connected")
status_label = ttk.Label(controls, textvariable=status_var)
status_label.pack(side="left", padx=15)

# Surface canvas
canvas = tk.Canvas(
    root,
    background="#f8f8f8",
    highlightthickness=0
)
canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

canvas.bind("<Configure>", lambda event: redraw())

# Start serial polling
root.after(20, read_serial)

root.mainloop()
