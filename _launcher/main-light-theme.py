import configparser
import os
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from datetime import datetime
from tkinter import PhotoImage, messagebox, ttk

import netifaces as ni
import pytz
import requests

import random, string;
from dotenv import load_dotenv

def buttonStartPressed():
    clear_db_states_file()

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

    if openNetworkVar.get():
        openNetworkMode = 1
    else:
        openNetworkMode = 0
            
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

    if remoteVideoAndCvVar.get():
        remoteVideoAndCv = 1
    else:
        remoteVideoAndCv = 0
    remoteVideoAndCvServer = remoteVideoAndCvServerEntry.get()
    remoteVideoAndCvPort = remoteVideoAndCvPortEntry.get()

    
    if isNvidiaRuntimeActive():
        nvidiaAvailable = 1
    else:
        nvidiaAvailable = 0


    if not openNetworkMode:
        algHost = netIp if config.get("Settings", "ALG_HOST") == "NET_IP" else config.get("Settings", "ALG_HOST")
        ccdHost = netIp if config.get("Settings", "CCD_HOST") == "NET_IP" else config.get("Settings", "CCD_HOST")
        cvHost = netIp if config.get("Settings", "CV_HOST") == "NET_IP" else config.get("Settings", "CV_HOST")
        cvnHost = netIp if config.get("Settings", "CVN_HOST") == "NET_IP" else config.get("Settings", "CVN_HOST")
        dbHost = netIp if config.get("Settings", "DB_HOST") == "NET_IP" else config.get("Settings", "DB_HOST")
        lscHost = netIp if config.get("Settings", "LSC_HOST") == "NET_IP" else config.get("Settings", "LSC_HOST")
        geoHost = netIp if config.get("Settings", "GEO_HOST") == "NET_IP" else config.get("Settings", "GEO_HOST")
        mavHost = netIp if config.get("Settings", "MAV_HOST") == "NET_IP" else config.get("Settings", "MAV_HOST")
        nginxHost = netIp if config.get("Settings", "NGINX_HOST") == "NET_IP" else config.get("Settings", "NGINX_HOST")
        odmHost = netIp if config.get("Settings", "ODM_HOST") == "NET_IP" else config.get("Settings", "ODM_HOST")
        rosHost = netIp if config.get("Settings", "ROS_HOST") == "NET_IP" else config.get("Settings", "ROS_HOST")
        rtmpHost = netIp if config.get("Settings", "RTMP_HOST") == "NET_IP" else config.get("Settings", "RTMP_HOST")
        webHost = netIp if config.get("Settings", "WEB_HOST") == "NET_IP" else config.get("Settings", "WEB_HOST")
        wsHost = netIp if config.get("Settings", "WS_HOST") == "NET_IP" else config.get("Settings", "WS_HOST")
        wsiHost = netIp if config.get("Settings", "WSI_HOST") == "NET_IP" else config.get("Settings", "WSI_HOST")
        wsmHost = netIp if config.get("Settings", "WSM_HOST") == "NET_IP" else config.get("Settings", "WSM_HOST")
        kcHost = netIp if config.get("Settings", "KC_HOST") == "NET_IP" else config.get("Settings", "KC_HOST")
        subnet = config.get("Settings", "SUBNET")
        gateway = config.get("Settings", "GATEWAY")
        algIp = netIp if config.get("Settings", "ALG_IP") == "NET_IP" else config.get("Settings", "ALG_IP")
        ccdIp = netIp if config.get("Settings", "CCD_IP") == "NET_IP" else config.get("Settings", "CCD_IP")
        cvIp = netIp if config.get("Settings", "CV_IP") == "NET_IP" else config.get("Settings", "CV_IP")
        cvnIp = netIp if config.get("Settings", "CVN_IP") == "NET_IP" else config.get("Settings", "CVN_IP")
        dbIp = netIp if config.get("Settings", "DB_IP") == "NET_IP" else config.get("Settings", "DB_IP")
        geoIp = netIp if config.get("Settings", "GEO_IP") == "NET_IP" else config.get("Settings", "GEO_IP")
        lscIp = netIp if config.get("Settings", "LSC_IP") == "NET_IP" else config.get("Settings", "LSC_IP")
        mavIp = netIp if config.get("Settings", "MAV_IP") == "NET_IP" else config.get("Settings", "MAV_IP")
        nginxIp = netIp if config.get("Settings", "NGINX_IP") == "NET_IP" else config.get("Settings", "NGINX_IP")
        odmIp = netIp if config.get("Settings", "ODM_IP") == "NET_IP" else config.get("Settings", "ODM_IP")
        rosIp = netIp if config.get("Settings", "ROS_IP") == "NET_IP" else config.get("Settings", "ROS_IP")
        rtmpIp = netIp if config.get("Settings", "RTMP_IP") == "NET_IP" else config.get("Settings", "RTMP_IP")
        webIp = netIp if config.get("Settings", "WEB_IP") == "NET_IP" else config.get("Settings", "WEB_IP")
        wsIp = netIp if config.get("Settings", "WS_IP") == "NET_IP" else config.get("Settings", "WS_IP")
        wsiIp = netIp if config.get("Settings", "WSI_IP") == "NET_IP" else config.get("Settings", "WSI_IP")
        wsmIp = netIp if config.get("Settings", "WSM_IP") == "NET_IP" else config.get("Settings", "WSM_IP")
        kcIp = netIp if config.get("Settings", "KC_IP") == "NET_IP" else config.get("Settings", "KC_IP")
    else:
        algHost = netIp
        ccdHost = netIp
        cvHost = netIp
        cvnHost = netIp
        dbHost = netIp
        lscHost = netIp
        geoHost = netIp
        mavHost = netIp
        nginxHost = netIp
        odmHost = netIp
        rosHost = netIp
        rtmpHost = netIp
        webHost = netIp
        wsHost = netIp
        wsiHost = netIp
        wsmHost = netIp
        kcHost = netIp
        subnet = "127.0.0.1"
        gateway = "127.0.0.1"
        algIp = netIp
        ccdIp = netIp
        cvIp = netIp
        cvnIp = netIp
        dbIp = netIp
        geoIp = netIp
        lscIp = netIp
        mavIp = netIp
        nginxIp = netIp
        odmIp = netIp
        rosIp = netIp
        rtmpIp = netIp
        webIp = netIp
        wsIp = netIp
        wsiIp = netIp
        wsmIp = netIp
        kcIp = netIp

    dbWhitelist = config.get("Settings", "DB_WHITELIST")



    ########################
    ### UPDATE .env FILE ###
    ########################
 
    modifyEnvVariable(envFilePath, "ALG_HOST", algHost)
    modifyEnvVariable(envFilePath, "CCD_HOST", ccdHost)
    modifyEnvVariable(envFilePath, "CV_HOST", cvHost)
    modifyEnvVariable(envFilePath, "CVN_HOST", cvnHost)
    modifyEnvVariable(envFilePath, "DB_HOST", dbHost)
    modifyEnvVariable(envFilePath, "LSC_HOST", lscHost)
    modifyEnvVariable(envFilePath, "GEO_HOST", geoHost)
    modifyEnvVariable(envFilePath, "MAV_HOST", mavHost)
    modifyEnvVariable(envFilePath, "NGINX_HOST", nginxHost)
    modifyEnvVariable(envFilePath, "ODM_HOST", odmHost)
    modifyEnvVariable(envFilePath, "ROS_HOST", rosHost)
    modifyEnvVariable(envFilePath, "RTMP_HOST", rtmpHost)
    modifyEnvVariable(envFilePath, "WEB_HOST", webHost)
    modifyEnvVariable(envFilePath, "WS_HOST", wsHost)
    modifyEnvVariable(envFilePath, "WSI_HOST", wsiHost)
    modifyEnvVariable(envFilePath, "WSM_HOST", wsmHost)
    modifyEnvVariable(envFilePath, "KC_HOST", kcHost)
    modifyEnvVariable(envFilePath, "SUBNET", subnet)
    modifyEnvVariable(envFilePath, "GATEWAY", gateway)
    modifyEnvVariable(envFilePath, "DB_WHITELIST", dbWhitelist)
    modifyEnvVariable(envFilePath, "ALG_IP", algIp)
    modifyEnvVariable(envFilePath, "CCD_IP", ccdIp)
    modifyEnvVariable(envFilePath, "CV_IP", cvIp)
    modifyEnvVariable(envFilePath, "CVN_IP", cvnIp)
    modifyEnvVariable(envFilePath, "DB_IP", dbIp)
    modifyEnvVariable(envFilePath, "GEO_IP", geoIp)
    modifyEnvVariable(envFilePath, "LSC_IP", lscIp)
    modifyEnvVariable(envFilePath, "MAV_IP", mavIp)
    modifyEnvVariable(envFilePath, "NGINX_IP", nginxIp)
    modifyEnvVariable(envFilePath, "ODM_IP", odmIp)
    modifyEnvVariable(envFilePath, "ROS_IP", rosIp)
    modifyEnvVariable(envFilePath, "RTMP_IP", rtmpIp)
    modifyEnvVariable(envFilePath, "WEB_IP", webIp)
    modifyEnvVariable(envFilePath, "WS_IP", wsIp)
    modifyEnvVariable(envFilePath, "WSI_IP", wsiIp)
    modifyEnvVariable(envFilePath, "WSM_IP", wsmIp)
    modifyEnvVariable(envFilePath, "KC_IP", kcIp)
    modifyEnvVariable(envFilePath, "NET_IP", netIp)
    modifyEnvVariable(envFilePath, "DEBUG", debugMode)
    modifyEnvVariable(envFilePath, "OPEN_NETWORK", openNetworkMode)
    modifyEnvVariable(envFilePath, "STREAM_CAPTURE_FPS", streamFps)
    modifyEnvVariable(envFilePath, "COMPUTER_VISION_FPS", cvFps)
    modifyEnvVariable(envFilePath, "NVIDIA_AVAILABLE", nvidiaAvailable)
    modifyEnvVariable(envFilePath, "VIDEO_AND_CV_REMOTE", remoteVideoAndCv)
    modifyEnvVariable(envFilePath, "VIDEO_AND_CV_SERVER", remoteVideoAndCvServer)
    modifyEnvVariable(envFilePath, "VIDEO_AND_CV_PORT", remoteVideoAndCvPort)
    modifyEnvVariable(envFilePath, "VERSION", config.get("About", "VERSION"))

    # keycloak
    if config.get("Keycloak", "KEYCLOAK_IP") == "NET_IP":
        modifyEnvVariable(envFilePath, "KEYCLOAK_IP", netIp)

    # kafka
    if config.get("Settings", "KAFKA_LOCAL_IP") == "NET_IP":
        modifyEnvVariable(envFilePath, "KAFKA_LOCAL_IP", netIp)

    writeToInfobox("[INFO]: The platform is starting...")
    disableDropDownList(ipDropdown)
    disableDropDownList(streamFpsDropdown)
    disableDropDownList(cvFpsDropdown)
    disableDropDownList(timezoneDropdown)
    disableButton(buttonStart)
    disableButton(debugCheckbox)
    disableButton(openNetworkCheckbox)
    disableButton(remoteVideoAndCvCheckbox)
    disableButton(remoteVideoAndCvServerEntry)
    disableButton(remoteVideoAndCvPortEntry)
    disableButton(buttonStop)
    disableButton(buttonClearData)
    disableButton(buttonTools)
    hideCredentialFields()


    # check if kafka should be local and active and if so, start it
    load_dotenv(envFilePath)  # reload the environment variables from the .env file

    kafkaActive = os.getenv("KAFKA_ACTIVE", "0")
    kafkaLocal = os.getenv("KAFKA_LOCAL", "0")

    if kafkaActive == "1" and kafkaLocal == "1":
        kafkaStartThread = threading.Thread(target=startKafka, args=(netIp,))
        kafkaStartThread.start()

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


def updateKafkaAdvertisedListenersYml(_netIp):
    # KAFKA_ADVERTISED_LISTENERS: HOST://_netIp:9092,DOCKER://kafka:9093
    kafka_compose_path = os.path.join(getParentDirectory(), 'kafka-broker', 'docker-compose.yml')
    try:
        with open(kafka_compose_path, 'r') as file:
            lines = file.readlines()
        new_lines = []
        for line in lines:
            if "KAFKA_ADVERTISED_LISTENERS" in line and "HOST://" in line:
                # Replace the IP after HOST:// and before :9092
                import re
                new_line = re.sub(r'HOST://([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+):9092', f'HOST://{_netIp}:9092', line)
                new_lines.append(new_line)
            else:
                new_lines.append(line)
        with open(kafka_compose_path, 'w') as file:
            file.writelines(new_lines)
    except Exception as e:
        writeToInfobox(f"[ERROR]: Could not update Kafka docker-compose.yml: {e}")


def startKafka(_netIp):
    try:
        writeToInfobox(f"[INFO]: Starting Kafka on {_netIp}...")
        # change the IP address of "HOST:..." in the kafka-broker docker-compose.yml
        updateKafkaAdvertisedListenersYml(_netIp)
        process = subprocess.Popen(
            f"cd {os.path.join(getParentDirectory(), 'kafka-broker')} && docker compose -f docker-compose.yml up -d",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Redirect stderr to stdout
            text=True
        )
        # Start a thread to capture and display the output in real-time
        outputThread = threading.Thread(target=printSubprocessOutputToInfobox, args=(process, infobox))
        outputThread.start()
        # Wait for the process to complete
        while process.poll() is None:
            writeToInfobox("[INFO]: Waiting for Kafka to start...\n")
            time.sleep(1)
        outputThread.join()

        # run the kafka topic creation script kafka_create_topics.py
        writeToInfobox("[INFO]: Creating Kafka topics...")
        process = subprocess.Popen(
            f"cd {getParentDirectory()} && python3 kafka_create_topics.py",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        outputThread = threading.Thread(target=printSubprocessOutputToInfobox, args=(process, infobox))
        outputThread.start()
        process.wait()
        outputThread.join()
    except Exception as e:
        writeToInfobox(f"[ERROR]: Error starting Kafka: {e}")


def startDockerContainers():
    time.sleep(5)  # wait for Kafka to start
    try:
        dockerFileName="docker-compose.yml"
        if(openNetworkVar.get()):
            dockerFileName="docker-compose.openNetwork.yml"

        # platform containers
        process = subprocess.Popen(
            f"cd {getParentDirectory()} && docker compose -f {dockerFileName} up -d",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Redirect stderr to stdout
            text=True
        )
        # Start a thread to capture and display the output in real-time
        outputThread = threading.Thread(target=printSubprocessOutputToInfobox, args=(process, infobox))
        outputThread.start()

        # keycloak containers
        process2 = subprocess.Popen(
            f"cd {os.path.join(getParentDirectory(), 'keycloak')} && docker compose -f docker-compose.yml up -d",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Redirect stderr to stdout
            text=True
        )
        # Start a thread to capture and display the output in real-time
        outputThread2 = threading.Thread(target=printSubprocessOutputToInfobox, args=(process2, infobox))
        outputThread2.start()

        secondsWaiting = 0
        while not webServerIsUp() or not keycloakServerIsUp():
            secondsWaiting += 1
            if secondsWaiting >= 30:
                writeToInfobox("[INFO]: The platform is taking too long to start. Please check the logs for more information.")
                enableButton(buttonStop)
            time.sleep(1)

        # run the keycloak seeder script
        writeToInfobox("[INFO]: Updating Keycloak client...")
        process3 = subprocess.Popen(
            f"cd {getParentDirectory()} && python3 keycloak_seeder.py --client-only",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        outputThread3 = threading.Thread(target=printSubprocessOutputToInfobox, args=(process3, infobox))
        outputThread3.start()

        outputThread.join()
        outputThread2.join()
        outputThread3.join()

        ip = getEnvVariable("NET_IP")
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
        result2 = subprocess.run(
            f"cd {os.path.join(getParentDirectory(), 'keycloak')} && docker compose down",
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        result3 = subprocess.run(
            f"cd {os.path.join(getParentDirectory(), 'kafka-broker')} && docker compose down",
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        while webServerIsUp() or keycloakServerIsUp():
            time.sleep(1)

        writeToInfobox("[INFO]: The platform has stopped.")
        enableDropDownList(ipDropdown)
        enableDropDownList(streamFpsDropdown)
        enableDropDownList(cvFpsDropdown)
        enableButton(buttonStart)
        enableButton(debugCheckbox)
        enableButton(openNetworkCheckbox)
        enableButton(remoteVideoAndCvCheckbox)
        enableButton(remoteVideoAndCvServerEntry)
        enableButton(remoteVideoAndCvPortEntry)
        enableButton(buttonClearData)    
        enableButton(buttonTools)    

    except Exception as e:
        writeToInfobox("[ERROR]: Error stopping Docker containers...")
        writeToInfobox(f"{e}")


def resetData():
    try:
        stopDockerContainers()
        disableButton(buttonStart)
        disableButton(debugCheckbox)
        disableButton(openNetworkCheckbox)
        disableButton(remoteVideoAndCvCheckbox)
        disableButton(remoteVideoAndCvServerEntry)
        disableButton(remoteVideoAndCvPortEntry)
        disableButton(buttonClearData)
        disableButton(buttonTools)
        try:
            subprocess.run("docker rm -f db", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            subprocess.run(f"docker volume rm {os.path.basename(getParentDirectory())}_mysql-data", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        except:
            pass
        # os.remove(envFilePath) # delete .env file
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
    enableButton(openNetworkCheckbox)
    enableButton(remoteVideoAndCvCheckbox)
    enableButton(remoteVideoAndCvServerEntry)
    enableButton(remoteVideoAndCvPortEntry)


def webServerIsUp():
    ip = getEnvVariable("NET_IP")
    port = getEnvVariable("NGINX_PORT")
    try:
        response = requests.get(f"http://{ip}:{port}", timeout=1)
        if response.status_code == 200:
            return True
        else:
            return False
    except:
        return False


def keycloakServerIsUp():
    ip = getEnvVariable("NET_IP")
    port = getEnvVariable("NGINX_PORT")
    try:
        response = requests.get(f"http://{ip}:{port}", timeout=1)
        if response.status_code == 200:
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


def modifyEnvVariable(_envFilePath, _key, _newValue):
    with open(_envFilePath, 'r') as file:
        lines = file.readlines()
    modifiedLines = []
    for line in lines:
        parts = line.strip().split('=')
        if len(parts) == 2 and parts[0] == _key:
            modifiedLines.append(f"{_key}={_newValue}\n")
        else:
            modifiedLines.append(line)
    with open(_envFilePath, 'w') as file:
        file.writelines(modifiedLines)


def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory


def getParentDirectory():
    parentDdirectory = os.path.dirname(getCurrentDirectory())
    return parentDdirectory
    

def envFileExists(_envFilePath): 
    return os.path.isfile(_envFilePath)


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
        print("File not found.")
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
    if envFileExists(envFilePath):
        localIps.append(getEnvVariable("NET_IP"))
    interfaces = ni.interfaces()    # get a list of all network interfaces
    # iterate through the interfaces and get their IP addresses
    for interface in interfaces:
        if "docker" not in interface and "br" not in interface:
            addresses = ni.ifaddresses(interface)
            if ni.AF_INET in addresses:
                ipInfo = addresses[ni.AF_INET]
                for ip in ipInfo:
                    if ip["addr"] != "127.0.0.1":
                        localIps.append(ip['addr'])
    return localIps


def getTimezones():
    # Get all timezones and filter out values that start with 'Etc/GMT' but are not exactly 'Etc/GMT'
    etcGmtValues = [tz for tz in pytz.all_timezones if tz.startswith('Etc/GMT') and tz != 'Etc/GMT' and tz != 'Etc/GMT-0' and tz != 'Etc/GMT+0' ]
    # Define a custom sorting key function
    def sortTimeZoneKey(tz):
        # Extract the numeric part from the timezone name
        return int(tz.replace('Etc/GMT', ''))
    # Sort the filtered values
    timezones = sorted(etcGmtValues, key=sortTimeZoneKey)
    return timezones

def getLocalTimezone():
    if time.daylight:
        offsetHour = time.altzone / 3600
    else:
        offsetHour = time.timezone / 3600
    return 'Etc/GMT%+d' % offsetHour


def getFpsOptions(param):
    fpsOptions = list(range(1, 16))
    if envFileExists(envFilePath):
        if(param == "stream"):
            fpsOptions.insert(0, getEnvVariable("STREAM_CAPTURE_FPS"))
        elif(param == "cv"):
            fpsOptions.insert(0, getEnvVariable("COMPUTER_VISION_FPS"))            
    return fpsOptions


def toggleRemoteVideoAndCvFields():
    if remoteVideoAndCvVar.get():
        remoteVideoAndCvServerLabel.grid()
        remoteVideoAndCvServerEntry.grid()
        remoteVideoAndCvPortLabel.grid()
        remoteVideoAndCvPortEntry.grid()
    else:
        remoteVideoAndCvServerLabel.grid_remove()
        remoteVideoAndCvServerEntry.grid_remove()
        remoteVideoAndCvPortLabel.grid_remove()
        remoteVideoAndCvPortEntry.grid_remove()

def readConfigFile(_configFile):
    c = configparser.ConfigParser()
    c.read(_configFile)
    return c



envFilePath = os.path.join(getParentDirectory(), '.env')
configPath = os.path.join(getParentDirectory(), 'config.ini')
config = readConfigFile(configPath)
keycloakEnvFilePath = os.path.join(getParentDirectory(), 'keycloak', '.env')

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
selectedTimezone = tk.StringVar(value=getLocalTimezone())
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

# open network checkbox
openNetworkVar = tk.BooleanVar()
if getEnvVariable("OPEN_NETWORK") == "1":
    openNetworkVar.set(True)
else:
    openNetworkVar.set(False)
openNetworkLabel = tk.Label(settingsFrame, text="Open Network Mode:")
openNetworkLabel.grid(row=3, column=0, padx=10, pady=5, sticky='e')
openNetworkCheckbox = tk.Checkbutton(settingsFrame, text="", variable=openNetworkVar)
openNetworkCheckbox.grid(row=3, column=1, padx=2, pady=5, sticky='w')

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

# remote video settings
remoteVideoAndCvVar = tk.BooleanVar()
if getEnvVariable("VIDEO_AND_CV_REMOTE") == "1":
    remoteVideoAndCvVar.set(True)
else:
    remoteVideoAndCvVar.set(False)

remoteVideoAndCvLabel = tk.Label(videoSettingsFrame, text="Remote Video Mode:")
remoteVideoAndCvLabel.grid(row=2, column=0, padx=0, pady=5, sticky='e')
remoteVideoAndCvCheckbox = tk.Checkbutton(videoSettingsFrame, text="", variable=remoteVideoAndCvVar, command=toggleRemoteVideoAndCvFields)
remoteVideoAndCvCheckbox.grid(row=2, column=1, padx=10, pady=5, sticky='w')

# remote video ip
remoteVideoAndCvServerVar = getEnvVariable("VIDEO_AND_CV_SERVER") or ""
remoteVideoAndCvServerLabel = tk.Label(videoSettingsFrame, text="Remote Video Server:")
remoteVideoAndCvServerLabel.grid(row=3, column=0, padx=0, pady=5, sticky='e')
remoteVideoAndCvServerEntry = tk.Entry(videoSettingsFrame)
remoteVideoAndCvServerEntry.insert(0, remoteVideoAndCvServerVar)
remoteVideoAndCvServerEntry.grid(row=3, column=1, padx=10, pady=5)

# remote video port
remoteVideoAndCvPortVar = getEnvVariable("VIDEO_AND_CV_PORT") or ""
remoteVideoAndCvPortLabel = tk.Label(videoSettingsFrame, text="Remote Video Port:")
remoteVideoAndCvPortLabel.grid(row=4, column=0, padx=0, pady=5, sticky='e')
remoteVideoAndCvPortEntry = tk.Entry(videoSettingsFrame)
remoteVideoAndCvPortEntry.insert(0, remoteVideoAndCvPortVar)
remoteVideoAndCvPortEntry.grid(row=4, column=1, padx=10, pady=5)

# toggle remote video and cv fields
toggleRemoteVideoAndCvFields()

# # database fields
# databaseUsernameLabel = tk.Label(databaseFrame, text="Username:")
# databaseUsernameLabel.grid(row=0, column=0, padx=10, pady=2, sticky='e')
# databaseUsernameEntry = tk.Entry(databaseFrame)
# databaseUsernameEntry.grid(row=0, column=1, padx=10, pady=2)

# databasePasswordLabel = tk.Label(databaseFrame, text="Password:")
# databasePasswordLabel.grid(row=1, column=0, padx=10, pady=2, sticky='e')
# databasePasswordEntry = tk.Entry(databaseFrame, show="*")
# databasePasswordEntry.grid(row=1, column=1, padx=10, pady=2)

# # web admin fields
# webAdminUsernameLabel = tk.Label(webAdminFrame, text="Username:")
# webAdminUsernameLabel.grid(row=0, column=0, padx=10, pady=2, sticky='e')
# webAdminUsernameEntry = tk.Entry(webAdminFrame)
# webAdminUsernameEntry.grid(row=0, column=1, padx=10, pady=2)

# webAdminPasswordLabel = tk.Label(webAdminFrame, text="Password:")
# webAdminPasswordLabel.grid(row=1, column=0, padx=10, pady=2, sticky='e')
# webAdminPasswordEntry = tk.Entry(webAdminFrame, show="*")
# webAdminPasswordEntry.grid(row=1, column=1, padx=10, pady=2)

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



if envFileExists(envFilePath):
    hideCredentialFields()
    disableDropDownList(timezoneDropdown)
    if webServerIsUp():
        writeToInfobox(f"[INFO]: The platform is running at {getEnvVariable('NET_IP')}")
        disableButton(buttonStart)
        disableButton(debugCheckbox)
        disableButton(openNetworkCheckbox)
        disableButton(remoteVideoAndCvCheckbox)
        disableButton(remoteVideoAndCvServerEntry)
        disableButton(remoteVideoAndCvPortEntry)
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
        enableButton(openNetworkCheckbox)
        enableButton(remoteVideoAndCvCheckbox)
        enableButton(remoteVideoAndCvServerEntry)
        enableButton(remoteVideoAndCvPortEntry)
        disableButton(buttonStop)
        enableButton(buttonClearData)
        enableButton(buttonTools)

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


else:
    # showCredentialsFields()
    writeToInfobox("[INFO]: Welcome to AIDERS.")
    writeToInfobox("\n\n[WARN]: No environment files found. Please run the '_scripts/create_env_files.py' script or contact your system administrator.")
    settingsFrame.pack_forget()
    videoSettingsFrame.pack_forget()
    disableButton(buttonStart)
    disableButton(debugCheckbox)
    disableButton(openNetworkCheckbox)
    disableButton(remoteVideoAndCvCheckbox)
    disableButton(remoteVideoAndCvServerEntry)
    disableButton(remoteVideoAndCvPortEntry)
    disableButton(buttonStop)
    disableButton(buttonClearData)
    disableButton(buttonTools)



# Run the main event loop
window.mainloop()