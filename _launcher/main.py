import os
import time
import subprocess
import threading
import netifaces as ni
import configparser
import webbrowser
import pytz
from datetime import datetime

import tkinter as tk
from tkinter import ttk
from tkinter import PhotoImage
from tkinter import messagebox


def buttonStartPressed():
    clear_db_states_file()
    shouldCreateEnvFile = not envFileExists()

    # get values from the input fields
    netIp = ipDropdown.get()
    if (netIp == ""):
        writeToInfobox("[ERROR]: Please enter an IP address.")
        return
    
    timezone = timezoneDropdown.get()
    if (timezone == ""):
        writeToInfobox("[ERROR]: Please select a timezone.")
        return
    
    if debugVar.get():
        debugMode = 1
    else:
        debugMode = 0
            
    streamFps = streamFpsDropdown.get()
    if (streamFps == "" or not streamFps.isdigit()):
        writeToInfobox("[ERROR]: Please select a stream capture framerate.")
        return
    
    cvFps = cvFpsDropdown.get()
    if (cvFps == "" or not cvFps.isdigit()):
        writeToInfobox("[ERROR]: Please select a computer vision framerate.")
        return
    
    if(cvFps > streamFps):
        writeToInfobox("[ERROR]: Stream capture FPS must be greater or equal than Computer Vision FPS.")
        return
            
    databaseUsername = databaseUsernameEntry.get()
    databasePassword = databasePasswordEntry.get()
    if (shouldCreateEnvFile and (databaseUsername == "" or databasePassword == "")):
        writeToInfobox("[ERROR]: Please enter valid Database credentials.")
        return

    webAdminUsername = webAdminUsernameEntry.get()
    webAdminPassword = webAdminPasswordEntry.get()
    if (shouldCreateEnvFile and (webAdminUsername == "" or webAdminPassword == "")):
        writeToInfobox("[ERROR]: Please enter valid Web Admin credentials.")
        return
    
    if isNvidiaRuntimeActive():
        nvidiaAvailable = 1
    else:
        nvidiaAvailable = 0

    webUrl = "http://" + netIp + ":" + config.get("Settings", "WEB_PORT")

    if shouldCreateEnvFile:
        # create .env file
        envData = {
            "VERSION": config.get("About", "VERSION"),
            "NET_IP": netIp,
            "SQL_HOST": "127.0.0.1",
            "SQL_PORT": config.get("Settings", "SQL_PORT"),
            "SQL_DATABASE": config.get("Settings", "SQL_DATABASE"),
            "SQL_USER": databaseUsername,
            "SQL_PASSWORD": databasePassword,
            "ADMIN_USER": webAdminUsername,
            "ADMIN_PASSWORD": webAdminPassword,
            "WEB_PORT": config.get("Settings", "WEB_PORT"),
            "WEB_URL": webUrl,
            "NGINX_PORT": config.get("Settings", "NGINX_PORT"),
            "ROS_API_PORT": config.get("Settings", "ROS_API_PORT"),
            "LSC_API_PORT": config.get("Settings", "LSC_API_PORT"),
            "CV_API_PORT": config.get("Settings", "CV_API_PORT"),
            "ALG_API_PORT": config.get("Settings", "ALG_API_PORT"),
            "MAV_API_PORT": config.get("Settings", "MAV_API_PORT"),
            "SSM_API_PORT": config.get("Settings", "SSM_API_PORT"),
            "SAFEML_API_PORT": config.get("Settings", "SAFEML_API_PORT"),
            "WS_PORT": config.get("Settings", "WS_PORT"),
            "ROS_MYSQL_CONNECTION_POOLS": config.get("Settings", "ROS_MYSQL_CONNECTION_POOLS"),
            "LSC_MYSQL_CONNECTION_POOLS": config.get("Settings", "LSC_MYSQL_CONNECTION_POOLS"),
            "CV_MYSQL_CONNECTION_POOLS": config.get("Settings", "CV_MYSQL_CONNECTION_POOLS"),
            "ALG_MYSQL_CONNECTION_POOLS": config.get("Settings", "ALG_MYSQL_CONNECTION_POOLS"),
            "MAV_MYSQL_CONNECTION_POOLS": config.get("Settings", "MAV_MYSQL_CONNECTION_POOLS"),
            "SSM_MYSQL_CONNECTION_POOLS": config.get("Settings", "SSM_MYSQL_CONNECTION_POOLS"),
            "SAFEML_MYSQL_CONNECTION_POOLS": config.get("Settings", "SAFEML_MYSQL_CONNECTION_POOLS"),
            "NVIDIA_AVAILABLE": nvidiaAvailable,
            "DEBUG": debugMode,
            "DJANGO_ALLOWED_HOSTS": "* localhost 127.0.0.1",
            "SECRET_KEY": "django-insecure-7u@(b_01go-msdw=*smjl0(4+02scu=&)m-(*6x%9+wp4tik$^",
            "SQL_ENGINE": "django.contrib.gis.db.backends.mysql",
            "TZ": timezone,
            "STREAM_CAPTURE_FPS": streamFps,
            "COMPUTER_VISION_FPS": cvFps,
        }
        with open(envFilePath, 'w') as f:
            for key, value in envData.items():
                f.write(f"{key}={value}\n")        
    else:
        modifyEnvVariable("NET_IP", netIp) # update NET_IP
        modifyEnvVariable("WEB_URL", webUrl) # update WEB_URL
        modifyEnvVariable("DEBUG", debugMode)
        modifyEnvVariable("STREAM_CAPTURE_FPS", streamFps)
        modifyEnvVariable("COMPUTER_VISION_FPS", cvFps)
        modifyEnvVariable("VERSION", config.get("About", "VERSION"))


    writeToInfobox("[INFO]: The platform is starting...")
    disableDropDownList(ipDropdown)
    disableDropDownList(streamFpsDropdown)
    disableDropDownList(cvFpsDropdown)
    disableDropDownList(timezoneDropdown)
    disableButton(buttonStart)
    disableButton(debugCheckbox)
    disableButton(buttonStop)
    disableButton(buttonClearData)
    disableButton(buttonTools)
    hideCredentialFields()

    dockerStartThread = threading.Thread(target=startDockerContainers, args=())
    dockerStartThread.start()


def buttonToolsPressed():
    writeToInfobox("[INFO]: Launching tools application...")
    monitorThread = threading.Thread(target=launchToolsApp, args=())
    monitorThread.start()


def launchToolsApp():
    subprocess.run(f"{getParentDirectory()}/_scripts/main.sh &", shell=True)


def buttonMonitorPressed():
    writeToInfobox("[INFO]: Launching monitoring application...")
    monitorThread = threading.Thread(target=launchMonitoringApp, args=())
    monitorThread.start()


def launchMonitoringApp():
    subprocess.run(f"{getParentDirectory()}/_scripts/monitor.sh &", shell=True)


def buttonSimulatorPressed():  
    writeToInfobox("[INFO]: Launching the simulator...")
    monitorThread = threading.Thread(target=launchSimulator, args=())
    monitorThread.start()


def launchSimulator():
    subprocess.run(f"{getParentDirectory()}/_scripts/simulator.sh &", shell=True)


def buttonStopPressed():
    confirmation = messagebox.askyesno("Confirmation", "Stop the platform?")
    if confirmation:
        disableButton(buttonStop)      
        writeToInfobox("[INFO]: The platform is being stopped. Please wait...")
        # stop the docker containers
        dockerStopThread = threading.Thread(target=stopDockerContainers, args=())
        dockerStopThread.start()


def buttonClearDataPressed():
    confirmation = messagebox.askyesno("Confirm data reset", "Are you sure you want to clear all the data?",
                                       icon='warning', 
                                       detail="All the data (including user accounts, operations, etc.) will be deleted. This action cannot be undone.")
    if confirmation:
        writeToInfobox("[INFO]: Clearing data. Please wait...")
        disableButton(buttonClearData)
        disableButton(buttonTools)
        resetDataThread = threading.Thread(target=resetData, args=())
        resetDataThread.start()


def startDockerContainers():
    try:
        process = subprocess.Popen(
            f"cd {getParentDirectory()} && docker compose up -d",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Redirect stderr to stdout
            text=True
        )

        # Start a thread to capture and display the output in real-time
        outputThread = threading.Thread(target=printSubprocessOutputToInfobox, args=(process, infobox))
        outputThread.start()


        while not webserverIsUp():
            time.sleep(1)

        outputThread.join()

        ip = getEnvVariable("NET_IP")
        if getEnvVariable("DEBUG") == "1":
            port = getEnvVariable("WEB_PORT")
        else:
            port = getEnvVariable("NGINX_PORT")
        writeToInfobox(f"[INFO]: The platform started successfully at {ip}")
        enableButton(buttonStop)
        webbrowser.open(f"http://{ip}:{port}")
        # webbrowser.open(f"http://aiders:{port}")

    except:
        writeToInfobox("[ERROR]: Error starting Docker containers...")



def printSubprocessOutputToInfobox(process, infobox):
    while process.poll() is None:
        line = process.stdout.readline()
        if line:
            infobox.insert(tk.END, line)
            infobox.see(tk.END)  # Scroll to the end
            window.update_idletasks()  # Update the GUI


def stopDockerContainers():
    try:
        result = subprocess.run(
            f"cd {getParentDirectory()} && docker compose down",
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        while webserverIsUp():
            time.sleep(1)
        writeToInfobox("[INFO]: The platform has stopped.")
        enableDropDownList(ipDropdown)
        enableDropDownList(streamFpsDropdown)
        enableDropDownList(cvFpsDropdown)
        enableButton(buttonStart)
        enableButton(debugCheckbox)
        enableButton(buttonClearData)    
        enableButton(buttonTools)    

    except:
        writeToInfobox("[ERROR]: Error stopping Docker containers...")


def resetData():
    try:
        stopDockerContainers()
        disableButton(buttonStart)
        disableButton(debugCheckbox)
        disableButton(buttonClearData)
        disableButton(buttonTools)
        try:
            subprocess.run("docker rm -f db", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            subprocess.run(f"docker volume rm {os.path.basename(getParentDirectory())}_mysql-data", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        except:
            pass
        os.remove(envFilePath) # delete .env file
        writeToInfobox("[INFO]: Database was deleted successfully.")
        writeToInfobox("[INFO]: IMPORTANT: To also delete all the media files you need root privileges. Run the following command to delete them:")
        writeToInfobox(f"sudo {getParentDirectory()}/_scripts/delete_media.sh")
    except Exception as e:
        writeToInfobox(f"[ERROR]: An error occured: {e}")
        return
    showCredentialsFields()
    enableDropDownList(timezoneDropdown)
    enableButton(buttonStart)
    enableButton(debugCheckbox)


def webserverIsUp():
    if getEnvVariable("DEBUG") == "1":
        ip = getEnvVariable("NET_IP")
    else:
        ip = "0.0.0.0"
    port = getEnvVariable("WEB_PORT")
    try:
        result = subprocess.run(["netstat", "-tuln"], capture_output=True, text=True, check=True)
        if f"{ip}:{port}" in result.stdout:
            return True
        else:
            return False
    except:
        return False


def getEnvVariable(_key):
    try:
        with open(envFilePath, 'r') as file:
            lines = file.readlines()
        for line in lines:
            parts = line.strip().split('=')
            if len(parts) == 2 and parts[0] == _key:
                return parts[1]
        return None
    except FileNotFoundError:
        return None


def modifyEnvVariable(_key, _newValue):
    with open(envFilePath, 'r') as file:
        lines = file.readlines()
    modifiedLines = []
    for line in lines:
        parts = line.strip().split('=')
        if len(parts) == 2 and parts[0] == _key:
            modifiedLines.append(f"{_key}={_newValue}\n")
        else:
            modifiedLines.append(line)
    with open(envFilePath, 'w') as file:
        file.writelines(modifiedLines)


def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory


def getParentDirectory():
    parentDdirectory = os.path.dirname(getCurrentDirectory())
    return parentDdirectory
    

def envFileExists(): 
    return os.path.isfile(envFilePath)


def hasNvidiaGpu():
    try:
        result = subprocess.run('lsmod | grep -qi nvidia', shell=True, capture_output=True, text=True)
        return result.returncode == 0
    except FileNotFoundError:
        return False


def hasNvidiaRuntime():
    try:
        subprocess.check_output(["dpkg", "-l", "nvidia-container-runtime"])
        return True
    except subprocess.CalledProcessError:
        return False


def isNvidiaRuntimeActive():
    try:
        with open(f"{getParentDirectory()}/docker-compose.yml", 'r') as file:
            content = file.read()
            if "runtime: nvidia" in content:
                return True
            else:
                return False
    except FileNotFoundError:
        print(f"File not found.")
        return False
    

def writeToInfobox(_text):
    infobox.insert(tk.END, f"\n{_text}")
    infobox.see(tk.END)    


def hideCredentialFields():
    databaseFrame.pack_forget()
    webAdminFrame.pack_forget()


def showCredentialsFields():
    databaseFrame.pack(fill="both", padx=12, pady=5)
    webAdminFrame.pack(fill="both", padx=12, pady=5)


def disableButton(_button):
    _button.config(state=tk.DISABLED)


def enableButton(_button):
    _button.config(state=tk.NORMAL)


def disableDropDownList(_list):
    _list.configure(state="disabled")


def enableDropDownList(_list):
    _list.configure(state="normal")


def clear_db_states_file():
    with open(f"{getParentDirectory()}/db_states.txt", 'w') as file:
        file.write("")


def getLocalIps():
    localIps = []
    if envFileExists():
        localIps.append(getEnvVariable("NET_IP"))
    interfaces = ni.interfaces()    # get a list of all network interfaces
    # iterate through the interfaces and get their IP addresses
    for interface in interfaces:
        if "docker" not in interface and "br" not in interface:
            addresses = ni.ifaddresses(interface)
            if ni.AF_INET in addresses:
                ipInfo = addresses[ni.AF_INET]
                for ip in ipInfo:
                    localIps.append(ip['addr'])
    return localIps


def getTimezones():
    # Create a list of timezone identifiers
    allTimezones = pytz.all_timezones
    # timezones = [tz for tz in allTimezones if tz.startswith('Europe/') or tz.startswith('Asia/')]
    timezones = [tz for tz in allTimezones if tz.startswith('Europe/')]
    if envFileExists():
        timezones.insert(0, getEnvVariable("TZ"))    
    return timezones


def getFpsOptions(param):
    fpsOptions = list(range(1, 16))
    if envFileExists():
        if(param == "stream"):
            fpsOptions.insert(0, getEnvVariable("STREAM_CAPTURE_FPS"))
        elif(param == "cv"):
            fpsOptions.insert(0, getEnvVariable("COMPUTER_VISION_FPS"))            
    return fpsOptions


def readConfigFile(_configFile):
    c = configparser.ConfigParser()
    c.read(_configFile)
    return c



envFilePath = os.path.join(getParentDirectory(), '.env')
configPath = os.path.join(getParentDirectory(), 'config.ini')
config = readConfigFile(configPath)

# create the main window
window = tk.Tk(className="AIDERS Launcher")
window.title("AIDERS Launcher")
icon = tk.PhotoImage(file=f"{getCurrentDirectory()}/logo-white.png")
window.iconphoto(False, icon)

iconDelete = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-delete.png")
iconTools = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-tools.png")
iconStart = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-start.png")
iconStop = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-stop.png")
iconMonitor = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-monitor.png")
iconSim = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon-sim.png")

# frames
topFrame = ttk.Frame(window)
topFrame.pack()

userInputFrame = ttk.Frame(window)
userInputFrame.pack_propagate(True)
userInputFrame.pack(fill="both", padx=0, pady=0)

settingsFrame = ttk.LabelFrame(userInputFrame, text="General Settings")
settingsFrame.pack(fill="both", padx=12, pady=5)

videoSettingsFrame = ttk.LabelFrame(userInputFrame, text="Video Settings")
videoSettingsFrame.pack(fill="both", padx=12, pady=5)

databaseFrame = ttk.LabelFrame(userInputFrame, text="Database Credentials")

webAdminFrame = ttk.LabelFrame(userInputFrame, text="Web Admin Credentials")

gpuStatusFrame = ttk.LabelFrame(window, text="GPU Status")
gpuStatusFrame.pack(fill="both", padx=12, pady=5)

# featuresFrame = ttk.LabelFrame(window, text="Features")
# featuresFrame.pack(fill="both", padx=12, pady=5)

infoFrame = ttk.Frame(window)
infoFrame.pack(fill="both", padx=12, pady=5)

buttonsFrame = ttk.Frame(window)
buttonsFrame.pack(padx=0, pady=2)

footerFrame = ttk.Frame(window)
footerFrame.pack(padx=0, pady=2)

# header image
image = PhotoImage(file=f"{getCurrentDirectory()}/header.png")
imageLabel = tk.Label(topFrame, image=image)
imageLabel.grid(row=0, column=0, padx=0, pady=0)

# net IP drop-down list
localIps = getLocalIps()
selectedIp = tk.StringVar(value=localIps[0])
ipLabel = tk.Label(settingsFrame, text="Net IP Address:")
ipLabel.grid(row=0, column=0, padx=10, pady=5, sticky='e')
ipDropdown = ttk.Combobox(settingsFrame, textvariable=selectedIp, values=localIps)
ipDropdown.grid(row=0, column=1, padx=10, pady=5)

# timezones drop-down list
timezones = getTimezones()
selectedTimezone = tk.StringVar(value=timezones[0])
timezoneLabel = tk.Label(settingsFrame, text="Timezone:")
timezoneLabel.grid(row=1, column=0, padx=10, pady=5, sticky='e')
timezoneDropdown = ttk.Combobox(settingsFrame, textvariable=selectedTimezone, values=timezones)
timezoneDropdown.grid(row=1, column=1, padx=10, pady=5)

# debug checkbox
debugVar = tk.BooleanVar()
if getEnvVariable("DEBUG") == "1":
    debugVar.set(True)
else:
    debugVar.set(False)
debugLabel = tk.Label(settingsFrame, text="Debug Mode:")
debugLabel.grid(row=2, column=0, padx=10, pady=5, sticky='e')
debugCheckbox = tk.Checkbutton(settingsFrame, text="", variable=debugVar)
debugCheckbox.grid(row=2, column=1, padx=2, pady=5, sticky='w')


# stream capture FPS selection
streamFps = getFpsOptions("stream")
selectedStreamFps = tk.StringVar(value=streamFps[0])
streamFpsLabel = tk.Label(videoSettingsFrame, text="Stream Capture FPS:")
streamFpsLabel.grid(row=0, column=0, padx=0, pady=5, sticky='e')
streamFpsDropdown = ttk.Combobox(videoSettingsFrame, textvariable=selectedStreamFps, values=streamFps)
streamFpsDropdown.grid(row=0, column=1, padx=10, pady=5)

# computer vision FPS selection
cvFps = getFpsOptions("cv")
selectedCvFps = tk.StringVar(value=cvFps[0])
cvFpsLabel = tk.Label(videoSettingsFrame, text="Computer Vision FPS:")
cvFpsLabel.grid(row=1, column=0, padx=0, pady=5, sticky='e')
cvFpsDropdown = ttk.Combobox(videoSettingsFrame, textvariable=selectedCvFps, values=cvFps)
cvFpsDropdown.grid(row=1, column=1, padx=10, pady=5)

# database fields
databaseUsernameLabel = tk.Label(databaseFrame, text="Username:")
databaseUsernameLabel.grid(row=0, column=0, padx=10, pady=2, sticky='e')
databaseUsernameEntry = tk.Entry(databaseFrame)
databaseUsernameEntry.grid(row=0, column=1, padx=10, pady=2)

databasePasswordLabel = tk.Label(databaseFrame, text="Password:")
databasePasswordLabel.grid(row=1, column=0, padx=10, pady=2, sticky='e')
databasePasswordEntry = tk.Entry(databaseFrame, show="*")
databasePasswordEntry.grid(row=1, column=1, padx=10, pady=2)

# web admin fields
webAdminUsernameLabel = tk.Label(webAdminFrame, text="Username:")
webAdminUsernameLabel.grid(row=0, column=0, padx=10, pady=2, sticky='e')
webAdminUsernameEntry = tk.Entry(webAdminFrame)
webAdminUsernameEntry.grid(row=0, column=1, padx=10, pady=2)

webAdminPasswordLabel = tk.Label(webAdminFrame, text="Password:")
webAdminPasswordLabel.grid(row=1, column=0, padx=10, pady=2, sticky='e')
webAdminPasswordEntry = tk.Entry(webAdminFrame, show="*")
webAdminPasswordEntry.grid(row=1, column=1, padx=10, pady=2)

# NVIDIA info message
nvidiaLabel = tk.Label(gpuStatusFrame, text="")
nvidiaLabel.pack(pady=2)
if hasNvidiaGpu():
    nvidiaLabel.config(text="NVIDIA GPU is available!", foreground="green")
else:
    nvidiaLabel.config(text="NVIDIA GPU is not available!", foreground="red")

nvidiaRuntimeLabel = tk.Label(gpuStatusFrame, text="")
nvidiaRuntimeLabel.pack(pady=2)
if hasNvidiaRuntime():
    nvidiaRuntimeLabel.config(text="NVIDIA container runtime is installed!", foreground="green")
else:
    nvidiaRuntimeLabel.config(text="NVIDIA container runtime is not installed!", foreground="red")

nvidiaRuntimeActiveLabel = tk.Label(gpuStatusFrame, text="")
nvidiaRuntimeActiveLabel.pack(pady=2)
if isNvidiaRuntimeActive():
    nvidiaRuntimeActiveLabel.config(text="NVIDIA container runtime is active!", foreground="green")
else:
    nvidiaRuntimeActiveLabel.config(text="NVIDIA container runtime is not active!", foreground="red")

# # features checkboxes
# dbFeatureVar = tk.BooleanVar()
# dbFeatureVar.set(True)
# dbFeatureCheckbox = tk.Checkbutton(featuresFrame, text="MySQL Database", variable=dbFeatureVar, state=tk.DISABLED)
# dbFeatureCheckbox.grid(row=0, column=0, padx=10, pady=1, stick='w')

# webFeatureVar = tk.BooleanVar()
# webFeatureVar.set(True)
# webFeatureCheckbox = tk.Checkbutton(featuresFrame, text="Web Interface", variable=webFeatureVar, state=tk.DISABLED)
# webFeatureCheckbox.grid(row=0, column=1, padx=10, pady=1, stick='w')

# rosFeatureVar = tk.BooleanVar()
# rosFeatureVar.set(True)
# rosFeatureCheckbox = tk.Checkbutton(featuresFrame, text="ROS Services", variable=rosFeatureVar, state=tk.DISABLED)
# rosFeatureCheckbox.grid(row=1, column=0, padx=10, pady=1, stick='w')

# rtmpFeatureVar = tk.BooleanVar()
# rtmpFeatureVar.set(True)
# rtmpFeatureCheckbox = tk.Checkbutton(featuresFrame, text="RTMP Server", variable=rtmpFeatureVar, state=tk.DISABLED)
# rtmpFeatureCheckbox.grid(row=1, column=1, padx=10, pady=1, stick='w')

# liveStreamFeatureVar = tk.BooleanVar()
# liveStreamFeatureVar.set(True)
# liveStreamFeatureCheckbox = tk.Checkbutton(featuresFrame, text="Live Stream", variable=liveStreamFeatureVar, state=tk.DISABLED)
# liveStreamFeatureCheckbox.grid(row=2, column=0, padx=10, pady=1, stick='w')

# computerVisionFeatureVar = tk.BooleanVar()
# computerVisionFeatureVar.set(True)
# computerVisionFeatureCheckbox = tk.Checkbutton(featuresFrame, text="Computer Vision", variable=computerVisionFeatureVar)
# computerVisionFeatureCheckbox.grid(row=2, column=1, padx=10, pady=1, stick='w')

# algorithmsFeatureVar = tk.BooleanVar()
# algorithmsFeatureVar.set(True)
# algorithmsFeatureCheckbox = tk.Checkbutton(featuresFrame, text="Algorithms", variable=algorithmsFeatureVar)
# algorithmsFeatureCheckbox.grid(row=3, column=0, padx=10, pady=1, stick='w')

# general info and error messages
infobox = tk.Text(infoFrame, wrap=tk.WORD, width=50, height=9)
infobox.configure(font=("Monospace", 8), fg="orange", bg="black")
infobox.pack(fill="both", expand=True)

# buttons
buttonClearData = tk.Button(buttonsFrame, image=iconDelete, command=buttonClearDataPressed)
# buttonClearData.grid(row=0, column=0, padx=2, pady=0, stick='ew')

buttonTools = tk.Button(buttonsFrame, image=iconTools, command=buttonToolsPressed)
buttonTools.grid(row=0, column=0, padx=2, pady=0, stick='ew')

buttonStop = tk.Button(buttonsFrame, image=iconStop, command=buttonStopPressed)
buttonStop.grid(row=0, column=1, padx=2, pady=0, stick='ew')

buttonStart = tk.Button(buttonsFrame, image=iconStart, command=buttonStartPressed)
buttonStart.grid(row=0, column=2, padx=2, pady=0, stick='ew')

buttonMonitor = tk.Button(buttonsFrame, image=iconMonitor, command=buttonMonitorPressed)
buttonMonitor.grid(row=0, column=3, padx=2, pady=0, stick='ew')

buttonSimulator = tk.Button(buttonsFrame, image=iconSim, command=buttonSimulatorPressed)
buttonSimulator.grid(row=0, column=4, padx=2, pady=0, stick='ew')

# footer
year = datetime.now().year
version = config.get("About", "VERSION")
footerText = f"v{version} \u00A9{year} KIOS C.O.E."
footerLabel = tk.Label(footerFrame, text=footerText)
footerLabel.pack(fill="both", expand=True)



if envFileExists():
    hideCredentialFields()
    disableDropDownList(timezoneDropdown)
    if webserverIsUp():
        writeToInfobox(f"[INFO]: The platform is running at {getEnvVariable('NET_IP')}")
        disableButton(buttonStart)
        disableButton(debugCheckbox)
        enableButton(buttonStop)
        disableButton(buttonClearData)
        disableButton(buttonTools)
        disableDropDownList(ipDropdown)
        disableDropDownList(streamFpsDropdown)
        disableDropDownList(cvFpsDropdown)
    else:
        writeToInfobox("[INFO]: The platform is stopped.")
        enableButton(buttonStart)
        enableButton(debugCheckbox)
        disableButton(buttonStop)
        enableButton(buttonClearData)  
        enableButton(buttonTools)  
else:
    showCredentialsFields()
    writeToInfobox("[INFO]: Welcome to AIDERS.")
    enableButton(buttonStart)
    enableButton(debugCheckbox)
    disableButton(buttonStop)
    disableButton(buttonClearData)
    disableButton(buttonTools)


if hasNvidiaGpu():
    writeToInfobox("\n[INFO]: NVIDIA GPU is available!")

    if hasNvidiaRuntime():
        writeToInfobox("[INFO]: NVIDIA container runtime is installed!")
        if isNvidiaRuntimeActive():
            writeToInfobox("[INFO]: NVIDIA container runtime is active!")
        else:
            writeToInfobox("\n[WARNING]: NVIDIA container runtime is not active! Run the following command to activate it:")
            writeToInfobox(f" {getParentDirectory()}/_scripts/nvidia_container_enable.sh")            
    else:
        writeToInfobox("\n[WARNING]: NVIDIA container runtime is not installed! Run the following command to install it:")
        writeToInfobox(f" {getParentDirectory()}/_scripts/nvidia_container_install.sh")

else:
    writeToInfobox("\n[WARNING]: NVIDIA GPU is not available! Computer vision will run on CPU.")


# Run the main event loop
window.mainloop()