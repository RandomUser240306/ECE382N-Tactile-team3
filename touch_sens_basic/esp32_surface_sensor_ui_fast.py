import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import deque

BAUD_RATE = 115200
GRAPH_CHANNELS = {
    0: ("#007f73", "Channel 0"),
    4: ("#3478a8", "Channel 4"),
    5: ("#b48216", "Channel 5"),
    6: ("#d85b36", "Channel 6"),
}
GRAPH_WINDOW_SECONDS = 10
GRAPH_HISTORY = deque(maxlen=1000)

data_queue = queue.Queue()

ser = None
serial_thread = None
stop_serial = threading.Event()

# ============================================================
# Serial reader
# ============================================================

def serial_reader():
    """Continuously read raw UART data in a background thread."""

    buffer = b""

    while not stop_serial.is_set():

        try:
            # Only read bytes that have actually arrived.
            n = ser.in_waiting

            if n == 0:
                continue

            buffer += ser.read(n)

            # Process every complete line currently in the buffer.
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)

                line = line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                # Put raw line into queue for debugging.
                data_queue.put((line, time.monotonic()))

        except Exception:
            break


def connect_serial():
    global ser, serial_thread

    disconnect_serial()

    port = port_var.get()

    try:
        ser = serial.Serial(
            port,
            BAUD_RATE,
            timeout=0
        )

        stop_serial.clear()

        serial_thread = threading.Thread(
            target=serial_reader,
            daemon=True
        )

        serial_thread.start()

        status_var.set(f"Connected: {port}")
        status_label.config(foreground="green")

    except Exception as e:
        ser = None
        status_var.set(f"Connection failed: {e}")
        status_label.config(foreground="red")


def disconnect_serial():
    global ser

    stop_serial.set()

    if ser is not None:
        try:
            ser.close()
        except:
            pass

    ser = None


# ============================================================
# Process received data
# ============================================================

def process_serial_data():

    newest_raw = None

    try:
        while True:
            raw, received_at = data_queue.get_nowait()
            newest_raw = raw

            try:
                fields = [field.strip() for field in raw.split(",")]
                if len(fields) < 10:
                    continue
                values = [float(value) for value in fields[:10]]
                GRAPH_HISTORY.append((received_at, values))
            except ValueError:
                pass

    except queue.Empty:
        pass

    if newest_raw is not None:
        raw_var.set(f"RX: {newest_raw}")

    draw_graph()

    root.after(10, process_serial_data)


def draw_graph():
    canvas.delete("all")

    width = canvas.winfo_width()
    height = canvas.winfo_height()
    left, top, right, bottom = 76, 58, width - 30, height - 54
    plot_width = right - left
    plot_height = bottom - top
    if plot_width <= 0 or plot_height <= 0:
        return

    now = time.monotonic()
    visible = [sample for sample in GRAPH_HISTORY if now - sample[0] <= GRAPH_WINDOW_SECONDS]
    low, high = 700, 1000

    canvas.create_text(
        left, 25, anchor="w", text="LIVE SENSOR READINGS  /  50 Hz",
        fill="#173b38", font=("Segoe UI", 12, "bold")
    )

    for index, (channel, (color, label)) in enumerate(GRAPH_CHANNELS.items()):
        legend_x = left + index * 140
        canvas.create_line(legend_x, 43, legend_x + 22, 43, fill=color, width=3)
        latest = visible[-1][1][channel] if visible else None
        legend_text = f"{label}  {latest:.0f}" if latest is not None else label
        canvas.create_text(
            legend_x + 28, 43, anchor="w", text=legend_text,
            fill="#293c3a", font=("Segoe UI", 9)
        )

    for tick in range(6):
        fraction = tick / 5
        y = top + plot_height * fraction
        value = high - (high - low) * fraction
        canvas.create_line(left, y, right, y, fill="#dce5e3", width=1)
        canvas.create_text(
            left - 10, y, anchor="e", text=f"{value:.0f}",
            fill="#60716f", font=("Segoe UI", 9)
        )

    for seconds_ago in range(GRAPH_WINDOW_SECONDS + 1):
        x = right - plot_width * seconds_ago / GRAPH_WINDOW_SECONDS
        canvas.create_line(x, top, x, bottom, fill="#edf1f0", width=1)
        canvas.create_text(
            x, bottom + 18, text=f"-{seconds_ago}s",
            fill="#60716f", font=("Segoe UI", 9)
        )

    canvas.create_line(left, top, left, bottom, fill="#829390", width=1)
    canvas.create_line(left, bottom, right, bottom, fill="#829390", width=1)
    canvas.create_text(
        20, top + plot_height / 2, text="Reading", angle=90,
        fill="#405451", font=("Segoe UI", 9)
    )

    if not visible:
        canvas.create_text(
            left + plot_width / 2, top + plot_height / 2,
            text="Connect to the ESP32 to view live channel data",
            fill="#60716f", font=("Segoe UI", 12)
        )
        return

    for channel, (color, _) in GRAPH_CHANNELS.items():
        points = []
        for received_at, values in visible:
            x = right - (now - received_at) * plot_width / GRAPH_WINDOW_SECONDS
            plotted_value = min(max(values[channel], low), high)
            y = bottom - (plotted_value - low) * plot_height / (high - low)
            points.extend((x, y))
        if len(points) >= 4:
            canvas.create_line(*points, fill=color, width=2, smooth=True)
        elif points:
            x, y = points
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)


# ============================================================
# GUI
# ============================================================

root = tk.Tk()

root.title("ESP32 Live Sensor Graph")
root.geometry("1100x720")

controls = ttk.Frame(root, padding=8)
controls.pack(fill="x")

ttk.Label(
    controls,
    text="Serial Port:"
).pack(side="left")

port_var = tk.StringVar()

ports = [
    p.device
    for p in serial.tools.list_ports.comports()
]

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

status_var = tk.StringVar(
    value="Not connected"
)

status_label = ttk.Label(
    controls,
    textvariable=status_var
)

status_label.pack(side="left", padx=15)

# Debug: shows exactly what was last received.
raw_var = tk.StringVar(
    value="RX: waiting for data..."
)

ttk.Label(
    root,
    textvariable=raw_var,
    anchor="w"
).pack(
    fill="x",
    padx=10
)

canvas = tk.Canvas(
    root,
    background="#fbfcfa",
    highlightthickness=0
)

canvas.pack(
    fill="both",
    expand=True,
    padx=10,
    pady=10
)

canvas.bind("<Configure>", lambda event: draw_graph())

root.after(
    10,
    process_serial_data
)


def on_close():
    disconnect_serial()
    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    on_close
)

root.mainloop()