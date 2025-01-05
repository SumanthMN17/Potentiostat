import mplcursors
import tkinter as tk
from tkinter import messagebox, ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import threading
import time
import pandas as pd
import serial.tools.list_ports

# Serial communication object
ser = None
I_values = []  # List to store current values (I)
T_values = []  # List to store time values (T)

# Declare global variables for widgets
com_port_var = None
voltage_entry = None
duration_entry = None
fig = None
canvas = None
tree = None
x_min_entry = None
x_max_entry = None
y_min_entry = None
y_max_entry = None


# Function to save current (I) values to Excel using pandas
def save_to_excel(I_values):
    if len(I_values) == 0:
        print("No data to save.")
        return

    df = pd.DataFrame({'Current (I)': I_values, 'Time (T)': T_values})
    df.to_excel('current_data.xlsx', index=False)
    print("Current data saved to Excel.")

# Function to list available COM ports
def get_com_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports] if ports else ["No COM Ports Available"]

def serial_communication():
    global ser, I_values, T_values

    I_values.clear()
    T_values.clear()
    tree.delete(*tree.get_children())  # Clear existing entries in the table

    timeout_duration = 5
    last_data_time = None
    first_data_received = False
    buffer = ""

    while True:
        if ser.in_waiting > 0:
            try:
                incoming_data = ser.read(ser.in_waiting).decode()
                buffer += incoming_data

                # Process data only when both "I=" and ",T=" markers exist in the buffer
                while "I=" in buffer and ",T=" in buffer:
                    start_idx = buffer.find("I=")
                    end_idx = buffer.find(",T=", start_idx)

                    if end_idx != -1:
                        try:
                            I_part = buffer[start_idx + 2:end_idx].strip()
                            remaining_buffer = buffer[end_idx + 3:]
                            T_part_end = remaining_buffer.find(",")
                            T_part = remaining_buffer[:T_part_end].strip() if T_part_end != -1 else remaining_buffer.strip()

                            I = int(I_part) / 100000
                            T = int(T_part) / 1000

                            I_values.append(I)
                            T_values.append(T)
                            # print(I_values)
                            # print(T_values)
                            if not first_data_received:
                                first_data_received = True
                                last_data_time = time.time()

                            last_data_time = time.time()

                            buffer = buffer[end_idx + len(T_part) + 3:]
                            plot_graph()
                            update_table(T, I)  # Update table with each new value

                        except (ValueError, IndexError) as e:
                            print(f"Parsing error: {e}")
                            break
                    else:
                        break
            except Exception as e:
                print(f"Serial read error: {e}")

        if first_data_received and (time.time() - last_data_time > timeout_duration):
            print(f"No data received for {timeout_duration} seconds. Stopping collection.")
            break

        time.sleep(0.1)

    print(f"Data collection complete. {len(I_values)} data points collected.")
    save_to_excel(I_values)

# Function to send data over serial port
def send_data():
    global ser
    try:
        com_port = com_port_var.get()
        voltage = voltage_entry.get()
        duration = duration_entry.get()

        padded_voltage = voltage.zfill(4)
        padded_duration = str(int(duration.zfill(5)) * 1000)

        if ser and ser.is_open:
            ser.close()

        ser = serial.Serial(com_port, 115200, timeout=1)
        time.sleep(1)
        ser.flush()

        ser.write(f"{padded_voltage},{padded_duration}".encode())
        print(f"Sent data: Voltage={padded_voltage}, Duration={padded_duration}")

        threading.Thread(target=serial_communication).start()

    except serial.SerialException as e:
        messagebox.showerror("Error", f"Serial port error: {e}")
    except Exception as e:
        messagebox.showerror("Error", f"Send data error: {e}")


# Function to handle window closing
def on_closing():
    global ser, window
    if ser and ser.is_open:
        ser.close()
    print("Serial connection closed.")
    window.destroy()


# Function to plot the graph
def plot_graph():
    global fig, canvas, I_values, T_values
    if not I_values or not T_values:
        print("No data available for plotting.")
        return

    try:
        fig.clear()
        ax = fig.add_subplot(111)
        ax.plot(T_values, I_values, marker='o', markersize=0.1, markerfacecolor='white', linewidth=0.1, color='#00008B')

        ax.set_xlim(0, 70)  # Default x-axis from 0 to 70 seconds
        ax.set_ylim(-15, 15)  # Default y-axis from -1 to +1 μA

        ax.set_title("Current (I) vs Time Duration", fontsize=12)
        ax.set_xlabel("Time (Sec)", fontsize=10)
        ax.set_ylabel("Current (μA)", fontsize=10)
        ax.grid()

        cursor = mplcursors.cursor(ax.lines, hover=True)
        cursor.connect("add", lambda sel: sel.annotation.set_bbox(dict(facecolor='#0078D4', alpha=0.8)))

        canvas.draw()
    except ValueError as e:
        messagebox.showerror("Input Error", f"Error while plotting: {e}")


# Function to redraw the graph with updated limits
def redraw_graph():
    global fig, canvas, I_values, T_values
    if not I_values or not T_values:
        print("No data available for plotting.")
        return

    try:
        # Get axis limits from user input
        x_min = float(x_min_entry.get())
        x_max = float(x_max_entry.get())
        y_min = float(y_min_entry.get())
        y_max = float(y_max_entry.get())

        # Clear the figure and re-plot
        fig.clear()
        ax = fig.add_subplot(111)
        ax.plot(T_values, I_values, marker='o', markersize=2, markerfacecolor='white', linewidth=1, color='#1f77b4')

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

        ax.set_title("Current (I) vs Time Duration", fontsize=12)
        ax.set_xlabel("Time (Sec)", fontsize=10)
        ax.set_ylabel("Current (μA)", fontsize=10)
        ax.grid()

        cursor = mplcursors.cursor(ax.lines, hover=True)
        cursor.connect("add", lambda sel: sel.annotation.set_bbox(dict(facecolor='#0078D4', alpha=0.8)))

        canvas.draw()
    except ValueError as e:
        messagebox.showerror("Input Error", f"Invalid values for redrawing: {e}")


# Function to update the table with new data
def update_table(time, current):
    global tree
    tree.insert("", "end", values=(time, current))


# Function to create GUI
def create_gui():
    global voltage_entry, duration_entry, fig, canvas, com_port_var, tree, x_min_entry, x_max_entry, y_min_entry, y_max_entry, window

    window = tk.Tk()
    window.title("SMD Serial Communicator")
    window.geometry("1200x800")

    # Left frame for inputs and graph
    left_frame = tk.Frame(window, bg='white')
    left_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    # Right frame for the table
    right_frame = tk.Frame(window, bg='white')
    right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

    # Input fields in the left frame
    tk.Label(left_frame, text="Select COM Port").grid(row=0, column=0, padx=5, pady=5)
    com_ports = get_com_ports()
    com_port_var = tk.StringVar(left_frame)
    com_port_var.set(com_ports[0])
    tk.OptionMenu(left_frame, com_port_var, *com_ports).grid(row=0, column=1, padx=2, pady=2)

    tk.Label(left_frame, text="Voltage (mV)").grid(row=1, column=0, padx=2, pady=2)
    voltage_entry = tk.Entry(left_frame)
    voltage_entry.grid(row=1, column=1, padx=2, pady=2)

    tk.Label(left_frame, text="Time Duration (Sec)").grid(row=2, column=0, padx=2, pady=2)
    duration_entry = tk.Entry(left_frame)
    duration_entry.grid(row=2, column=1, padx=2, pady=2)

    tk.Button(left_frame, text="Send", command=send_data).grid(row=2, column=2, padx=5, pady=5)

    # Add axis limits inputs and redraw button
    tk.Label(left_frame, text="X Min").grid(row=5, column=0)
    x_min_entry = tk.Entry(left_frame)
    x_min_entry.grid(row=5, column=1)

    tk.Label(left_frame, text="X Max").grid(row=6, column=0)
    x_max_entry = tk.Entry(left_frame)
    x_max_entry.grid(row=6, column=1)

    tk.Label(left_frame, text="Y Min").grid(row=7, column=0)
    y_min_entry = tk.Entry(left_frame)
    y_min_entry.grid(row=7, column=1)

    tk.Label(left_frame, text="Y Max").grid(row=8, column=0)
    y_max_entry = tk.Entry(left_frame)
    y_max_entry.grid(row=8, column=1)

    tk.Button(left_frame, text="Redraw", command=redraw_graph).grid(row=9, column=1)

    # Add Matplotlib figure
    fig = Figure(figsize=(6, 4), dpi=100)
    canvas = FigureCanvasTkAgg(fig, master=left_frame)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.grid(row=4, column=0, columnspan=3)

    # Table in the right frame
    tree = ttk.Treeview(right_frame, columns=("Time", "Current"), show="headings", height=15)
    tree.heading("Time", text="Time (Sec)")
    tree.heading("Current", text="Current (μA)")
    tree.pack(fill="both", expand=True)

    window.protocol("WM_DELETE_WINDOW", on_closing)
    window.mainloop()


if __name__ == "__main__":
    create_gui()
