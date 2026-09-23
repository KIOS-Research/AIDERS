import tkinter as tk
from tkinter import ttk, messagebox, PhotoImage
import netifaces as ni
import subprocess
import time
import threading
import os
import requests
import json

def read_env_values():
    env_values = {}
    try:
        with open(f"{getParentDirectory()}/.env", 'r') as env_file:
            lines = env_file.readlines()
            for line in lines:
                key, value = line.strip().split('=')
                env_values[key] = value
    except FileNotFoundError:
        pass
    return env_values

def set_drone_parameter(drone_index, parameter, value):
    url = 'http://localhost/setDroneParameter'
    # ip_var.get()
    data = {
        "droneIndex": drone_index,
        "parameter": parameter,
        "value": value
    }
    headers = {'Content-Type': 'application/json'}

    response = requests.post(url, data=json.dumps(data), headers=headers)

    if response.status_code == 200:
        print('POST request successful')
    else:
        print('POST request failed with status code:', response.status_code)

def container_is_running(container_name):
    command = f"docker ps --format '{{{{.Names}}}}' | grep {container_name}"
    result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    return result.returncode == 0 and container_name in result.stdout

def update_field_visibility(parent_var, field_label, field_entry):
    if parent_var.get() == "0":
        field_label.grid_remove()
        field_entry.grid_remove()
    else:
        field_label.grid()
        field_entry.grid()

def update_dual_field_visibility(parent_var, field_label, field_entry, field2_label, field2_entry):
    if parent_var.get() == "0":
        field_label.grid_remove()
        field_entry.grid_remove()
        field2_label.grid_remove()
        field2_entry.grid_remove()        
    else:
        field_label.grid()
        field_entry.grid()
        field2_label.grid()
        field2_entry.grid()

def disable_button(_button):
    _button.config(state=tk.DISABLED)

def enable_button(_button):
    _button.config(state=tk.NORMAL)

def get_local_ips():
    local_ips = []
    interfaces = ni.interfaces()    # get a list of all network interfaces
    # iterate through the interfaces and get their IP addresses
    for interface in interfaces:
        if "docker" not in interface and "br" not in interface:
            addresses = ni.ifaddresses(interface)
            if ni.AF_INET in addresses:
                ipInfo = addresses[ni.AF_INET]
                for ip in ipInfo:
                    local_ips.append(ip['addr'])
    return local_ips

def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory

def getParentDirectory():
    parentDdirectory = os.path.dirname(getCurrentDirectory())
    return parentDdirectory

def start_simulator():
    try:
        ros_ip = ip_var.get()
        wsi_port = env_values.get("WSI_PORT", "8773")
        num_drones_ros = int(drones_ros_var.get())
        num_drones_ws = int(drones_ws_var.get())
        drone_freq = float(drone_freq_var.get())
        num_devices = int(devices_var.get())
        device_freq = float(device_freq_var.get())
        num_lora_masters = int(lora_masters_var.get())
        num_lora_clients = int(lora_clients_var.get())
        lora_freq = float(lora_freq_var.get())

        if liveStreamVar.get():
            live_stream = 1
        else:
            live_stream = 0


        if not (num_drones_ros <= 50) or not (0.1 <= drone_freq <= 10) or \
           not (0 <= num_devices <= 50) or not (0.1 <= device_freq <= 10):
            raise ValueError("Invalid input range")

        with open(f"{getParentDirectory()}/.env", 'w') as env_file:
            env_file.write(f"PLATFORM_IP={ros_ip}\n")
            env_file.write(f"WSI_PORT={wsi_port}\n")
            env_file.write(f"NUM_DRONES_ROS={num_drones_ros}\n")
            env_file.write(f"NUM_DRONES_WS={num_drones_ws}\n")
            env_file.write(f"DRONE_FREQ={drone_freq}\n")
            env_file.write(f"DRONE_LIVE_STREAM={live_stream}\n")
            env_file.write(f"NUM_DEVICES={num_devices}\n")
            env_file.write(f"DEVICE_FREQ={device_freq}\n")
            env_file.write(f"NUM_LORA_MASTERS={num_lora_masters}\n")
            env_file.write(f"NUM_LORA_CLIENTS={num_lora_clients}\n")
            env_file.write(f"LORA_FREQ={lora_freq}\n")

        write_to_infobox("\nThe simulator is starting...\n")
        subprocess.run([f"cd {getParentDirectory()} && docker compose up -d"], shell=True)

        disable_button(start_button)
        enable_button(stop_button)

    except ValueError as e:
        messagebox.showerror("Error", str(e))


def stop_simulator():
    disable_button(stop_button)
    write_to_infobox("\nThe simulator is being stopped...\n")
    stop_simulator_thread = threading.Thread(target=stop_simulator_process, args=())
    stop_simulator_thread.daemon = True
    stop_simulator_thread.start()    

def stop_simulator_process():
    subprocess.run([f"cd {getParentDirectory()} && docker compose down"], shell=True)
    infobox.config(state=tk.NORMAL)
    infobox.insert(tk.END, "\nThe simulator has stopped!\n")
    infobox.config(state=tk.DISABLED)
    infobox.see(tk.END)
    enable_button(start_button)

def update_infobox():
    last_position = 0
    while True:
        with open(f"{getParentDirectory()}/logs/app_logs.txt", 'r') as file:
            file.seek(last_position)  # Move to the last position read
            new_lines = file.readlines()
            if new_lines:
                last_position = file.tell()  # Update the last position read
                infobox.config(state=tk.NORMAL)
                for line in new_lines:
                    infobox.insert(tk.END, line)
                infobox.config(state=tk.DISABLED)
                infobox.see(tk.END)  # Scroll to the end
        time.sleep(0.5)

def write_to_infobox(_text):
    infobox.config(state=tk.NORMAL)
    infobox.insert(tk.END, _text)
    infobox.config(state=tk.DISABLED)
    infobox.see(tk.END)  # Scroll to the end

def clear_log_file():
    with open(f"{getParentDirectory()}/logs/app_logs.txt", 'w') as file:
        file.write("")



# Read values from .env file
env_values = read_env_values()
freq_values_list = ["0.1", "0.2", "0.3", "0.5", "1", "2", "3", "5", "8", "10"]

sim_api_url = "http://127.0.0.1:8991"

colours = {
    'green': '#99DD99',
    'red': '#FF9999',
    'orange': '#f0b781',
    'grey': '#CCCCCC',
}

app = tk.Tk(className="AIDERS Simulator")
app.title("AIDERS Simulator")
icon = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon.png")
app.iconphoto(False, icon)

iconStart = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-start.png")
iconStop = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-stop.png")
iconSend = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-send.png")
iconCrisis = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-crisis.png")

# header image
top_frame = ttk.Frame(app)
top_frame.grid(row=0, column=0, padx=2, pady=2, sticky="ew")
image = PhotoImage(file=f"{getCurrentDirectory()}/header2.png")
image_label = tk.Label(top_frame, image=image)
image_label.grid(row=0, column=0, padx=0, pady=0)

# Network Fieldset
network_frame = ttk.LabelFrame(app, text="Network")
network_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

local_ips = get_local_ips()
ip_label = tk.Label(network_frame, text="Net IP Address:")
ip_label.grid(row=0, column=0, sticky="ew")
ip_var = tk.StringVar(value=env_values.get("PLATFORM_IP", "127.0.0.1"))
ip_entry = ttk.Combobox(network_frame, textvariable=ip_var, values=local_ips)
ip_entry.grid(row=0, column=1, padx=5, pady=5)

# Drones Fieldset
drones_frame = ttk.LabelFrame(app, text="Drones")
drones_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

drones_ros_label = tk.Label(drones_frame, text="ROS Drones:")
drones_ros_label.grid(row=0, column=0, sticky="w")
drones_ros_var = tk.StringVar(value=env_values.get("NUM_DRONES_ROS", "0"))
drones_entry = ttk.Combobox(drones_frame, textvariable=drones_ros_var, values=list(map(str, range(51))))
drones_entry.grid(row=0, column=1, padx=5, pady=2)
# drones_entry.bind("<<ComboboxSelected>>", lambda event: update_field_visibility(drones_ros_var, drone_freq_label, drone_freq_entry))

drones_ws_label = tk.Label(drones_frame, text="Websocket Drones:")
drones_ws_label.grid(row=1, column=0, sticky="w")
drones_ws_var = tk.StringVar(value=env_values.get("NUM_DRONES_WS", "0"))
drones_entry = ttk.Combobox(drones_frame, textvariable=drones_ws_var, values=list(map(str, range(51))))
drones_entry.grid(row=1, column=1, padx=5, pady=2)

drone_freq_label = tk.Label(drones_frame, text="Transmit Freq (Hz):")
drone_freq_label.grid(row=2, column=0, sticky="w")
drone_freq_var = tk.StringVar(value=env_values.get("DRONE_FREQ", "0.1"))
drone_freq_entry = ttk.Combobox(drones_frame, textvariable=drone_freq_var, values=freq_values_list)
drone_freq_entry.grid(row=2, column=1, padx=5, pady=2)
# update_field_visibility(drones_ros_var, drone_freq_label, drone_freq_entry)

liveStreamVar = tk.BooleanVar()
liveStreamVar.set(True)
liveStreamCheckbox = tk.Checkbutton(drones_frame, text="Live Stream", variable=liveStreamVar)
liveStreamCheckbox.grid(row=3, column=1, padx=0, pady=2, stick='w')

# Devices Fieldset
devices_frame = ttk.LabelFrame(app, text="Devices")
devices_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

devices_label = tk.Label(devices_frame, text="Number of Devices:")
devices_label.grid(row=0, column=0, sticky="w")
devices_var = tk.StringVar(value=env_values.get("NUM_DEVICES", "0"))
devices_entry = ttk.Combobox(devices_frame, textvariable=devices_var, values=list(map(str, range(51))))
devices_entry.grid(row=0, column=1, padx=5, pady=2)
devices_entry.bind("<<ComboboxSelected>>", lambda event: update_field_visibility(devices_var, device_freq_label, device_freq_entry))

device_freq_label = tk.Label(devices_frame, text="Transmit Freq (Hz):")
device_freq_label.grid(row=1, column=0, sticky="w")
device_freq_var = tk.StringVar(value=env_values.get("DEVICE_FREQ", "0.1"))
device_freq_entry = ttk.Combobox(devices_frame, textvariable=device_freq_var, values=freq_values_list)
device_freq_entry.grid(row=1, column=1, padx=5, pady=2)
update_field_visibility(devices_var, device_freq_label, device_freq_entry)

# Lora Fieldset
lora_frame = ttk.LabelFrame(app, text="Lora")
lora_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")

lora_masters_label = tk.Label(lora_frame, text="Number of Masters:")
lora_masters_label.grid(row=0, column=0, sticky="w")
lora_masters_var = tk.StringVar(value=env_values.get("NUM_LORA_MASTERS", "0"))
lora_masters_entry = ttk.Combobox(lora_frame, textvariable=lora_masters_var, values=list(map(str, range(6))))
lora_masters_entry.grid(row=0, column=1, padx=5, pady=2)
lora_masters_entry.bind("<<ComboboxSelected>>", lambda event: update_dual_field_visibility(lora_masters_var, lora_clients_label, lora_clients_entry, lora_freq_label, lora_freq_entry))

lora_clients_label = tk.Label(lora_frame, text="Number of Clients:")
lora_clients_label.grid(row=1, column=0, sticky="w")
lora_clients_var = tk.StringVar(value=env_values.get("NUM_LORA_CLIENTS", "1"))
lora_clients_entry = ttk.Combobox(lora_frame, textvariable=lora_clients_var, values=list(map(str, range(1, 51))))
lora_clients_entry.grid(row=1, column=1, padx=5, pady=2)
update_field_visibility(lora_masters_var, lora_clients_label, lora_clients_entry)

lora_freq_label = tk.Label(lora_frame, text="Transmit Freq (Hz):")
lora_freq_label.grid(row=2, column=0, sticky="w")
lora_freq_var = tk.StringVar(value=env_values.get("LORA_FREQ", "0.1"))
lora_freq_entry = ttk.Combobox(lora_frame, textvariable=lora_freq_var, values=freq_values_list)
lora_freq_entry.grid(row=2, column=1, padx=5, pady=2)
update_field_visibility(lora_masters_var, lora_freq_label, lora_freq_entry)


def send_param_change_request():
    droneLabel = param_change_drone_var.get()
    droneIndex = DRONE_MAP.get(droneLabel, "")    
    parameter = param_change_parameter_var.get()
    value = int(param_change_value_var.get())
    url = f"{sim_api_url}/setDroneParameter"
    data = {
        "droneIndex": droneIndex,
        "parameter": parameter,
        "value": value
    }
    headers = {'Content-Type': 'application/json'}

    try:
        response = requests.post(url, data=json.dumps(data), headers=headers)
        if response.status_code == 200:
            write_to_infobox("\nParam Change request successful.")
        else:
            write_to_infobox(f"\nParam Change failed: {response.status_code}\n")
    except Exception as e:
        write_to_infobox(f"\nParam Change error: {e}\n")

DRONE_MAP = {
    "SIM_Alpha": 1,
    "SIM_Bravo": 2,
    "SIM_Charlie": 3,
    "SIM_Delta": 4
}

# Param Change request section
param_change_frame = ttk.LabelFrame(app, text="Change Drone Parameter")
param_change_frame.grid(row=5, column=0, padx=10, pady=5, sticky="ew")

param_change_drone_var = tk.StringVar()
param_change_drone_entry = ttk.Combobox(param_change_frame, textvariable=param_change_drone_var, values=list(DRONE_MAP.keys()), width=9)
param_change_drone_entry.grid(row=0, column=0, padx=2, pady=2)
param_change_drone_entry.current(0)

param_change_parameter_var = tk.StringVar()
param_change_parameter_entry = ttk.Combobox(param_change_frame, textvariable=param_change_parameter_var, values=["altitude", "heading", "gimbalAngle", "batteryPercentage", "demoAltitude", "demoHeading", "demoGimbalAngle"], width=16)
param_change_parameter_entry.grid(row=0, column=1, padx=2, pady=2)
param_change_parameter_entry.current(0)

param_change_value_var = tk.StringVar()
param_change_value_entry = tk.Entry(param_change_frame, textvariable=param_change_value_var, width=4)
param_change_value_entry.grid(row=0, column=3, padx=2, pady=2)

param_change_button = tk.Button(param_change_frame, image=iconSend, bg="lightgreen", command=send_param_change_request)
param_change_button.grid(row=0, column=4, padx=2, pady=5)


def send_crisis_generation_request():
    incident_id = crisis_incident_id_var.get()
    incident_type_label = crisis_incident_type_var.get()
    incident_type = CRISIS_TYPE_MAP.get(incident_type_label, "")
    severity_level_label = crisis_incident_severity_var.get()
    severity_level = CRISIS_SEVERITY_MAP.get(severity_level_label, "")

    url = f"{sim_api_url}/generateCrisisIncident"
    data = {
        "id": incident_id,
        "type": incident_type,
        "severity": severity_level
    }

    headers = {'Content-Type': 'application/json'}
    try:
        response = requests.post(url, data=json.dumps(data), headers=headers)
        if response.status_code == 200:
            write_to_infobox("\nCrisis Generation request successful.")
        else:
            write_to_infobox(f"\nCrisis Generation failed: {response.status_code}\n")
    except Exception as e:
        write_to_infobox(f"\nCrisis Generation error: {e}\n")

    # Increment the crisis_incident_id_var value
    try:
        current_id = int(crisis_incident_id_var.get())
        crisis_incident_id_var.set(str(current_id + 1))
    except ValueError:
        pass  # Ignore if not an integer
    

CRISIS_TYPE_MAP = {
    "Suspicious Activity": "SuspiciousActivity",
    "Human Trafficking": "Human_Trafficking",
    "Smuggling": "Smuggling",
    "Natural Disaster": "Natural_Disaster",
    "Terrorist Attack": "Terrorist_Attack"
}

CRISIS_SEVERITY_MAP = {
    "High": "High",
    "Medium": "Medium",
    "Low": "Low"
}

# Crisis Generation request section
crisis_generation_frame = ttk.LabelFrame(app, text="Generate Crisis Incident")
crisis_generation_frame.grid(row=6, column=0, padx=10, pady=5, sticky="ew")
crisis_incident_id_var = tk.StringVar(value="123")
crisis_incident_id_entry = tk.Entry(crisis_generation_frame, textvariable=crisis_incident_id_var, width=6)
crisis_incident_id_entry.grid(row=0, column=0, padx=2, pady=2)
crisis_incident_type_var = tk.StringVar(value=list(CRISIS_TYPE_MAP.keys())[0])
crisis_incident_type_entry = ttk.Combobox(crisis_generation_frame, textvariable=crisis_incident_type_var, values=list(CRISIS_TYPE_MAP.keys()), width=16)
crisis_incident_type_entry.grid(row=0, column=1, padx=2, pady=2)
# crisis_incident_action_var = tk.StringVar(value="Create")
# crisis_incident_action_entry = ttk.Combobox(crisis_generation_frame, textvariable=crisis_incident_action_var, values=["Create", "Update", "Alert", "Resolve"], width=7)
# crisis_incident_action_entry.grid(row=0, column=2, padx=2, pady=2)
crisis_incident_severity_var = tk.StringVar(value=list(CRISIS_SEVERITY_MAP.keys())[0])
crisis_incident_severity_entry = ttk.Combobox(crisis_generation_frame, textvariable=crisis_incident_severity_var, values=list(CRISIS_SEVERITY_MAP.keys()), width=7)
crisis_incident_severity_entry.grid(row=0, column=2, padx=2, pady=2)
crisis_generation_button = tk.Button(crisis_generation_frame, image=iconCrisis, bg="lightgreen", command=send_crisis_generation_request)
crisis_generation_button.grid(row=0, column=3, padx=3, pady=5)


# infobox
infobox = tk.Text(app, wrap=tk.WORD, width=54, height=9)
infobox.configure(font=("Monospace", 8), fg="orange", bg="black")
infobox.grid(row=7, column=0, pady=2)
clear_log_file()
update_infobox_thread = threading.Thread(target=update_infobox, args=())
update_infobox_thread.daemon = True  # The thread will be terminated when the program exits
update_infobox_thread.start()

# buttons
buttons_frame = ttk.Frame(app)
buttons_frame.grid(row=8, column=0, pady=5)

stop_button = tk.Button(buttons_frame, image=iconStop, command=stop_simulator)
stop_button.grid(row=0, column=0, padx=4)

start_button = tk.Button(buttons_frame, image=iconStart, command=start_simulator)
start_button.grid(row=0, column=1, padx=4)

if container_is_running("sim"):
    disable_button(start_button)
    write_to_infobox("\nThe simulator is running.")
else:
    disable_button(stop_button)
    write_to_infobox("\nThe simulator is stopped.")

app.mainloop()

