#!/usr/bin/env python3

import configparser
import os
import random, string;
from dotenv import load_dotenv


def createEnvFiles():

    shouldCreateEnvFile = not envFileExists(envFilePath)
    shouldCreateKeycloakEnvFile = not envFileExists(keycloakEnvFilePath)

    netIp = "127.0.0.1"
    timezone = "UTC"
    openNetworkMode = 0
    debugMode = 1
    streamFps = 5
    cvFps = 5
    remoteVideoAndCv = 0

    if isNvidiaRuntimeActive():
        nvidiaAvailable = 1
    else:
        nvidiaAvailable = 0

 
    databaseUsername = config.get("Settings", "DB_USER")
    databasePassword = ''.join(random.choices(string.ascii_letters + string.digits, k=16))
    webAdminUsername = config.get("Settings", "ADMIN_USER")
    webAdminPassword = ''.join(random.choices(string.ascii_letters + string.digits, k=16))


    algHost = netIp if config.get("Settings", "ALG_HOST") == "NET_IP" else config.get("Settings", "ALG_HOST")
    ccdHost = netIp if config.get("Settings", "CCD_HOST") == "NET_IP" else config.get("Settings", "CCD_HOST")
    cvHost = netIp if config.get("Settings", "CV_HOST") == "NET_IP" else config.get("Settings", "CV_HOST")
    cvnHost = netIp if config.get("Settings", "CVN_HOST") == "NET_IP" else config.get("Settings", "CVN_HOST")
    streamcvHost = netIp if config.get("Settings", "STREAMCV_HOST") == "NET_IP" else config.get("Settings", "STREAMCV_HOST")
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
    dridHost = netIp if config.get("Settings", "DRID_HOST") == "NET_IP" else config.get("Settings", "DRID_HOST")
    subnet = config.get("Settings", "SUBNET")
    gateway = config.get("Settings", "GATEWAY")
    algIp = netIp if config.get("Settings", "ALG_IP") == "NET_IP" else config.get("Settings", "ALG_IP")
    ccdIp = netIp if config.get("Settings", "CCD_IP") == "NET_IP" else config.get("Settings", "CCD_IP")
    cvIp = netIp if config.get("Settings", "CV_IP") == "NET_IP" else config.get("Settings", "CV_IP")
    cvnIp = netIp if config.get("Settings", "CVN_IP") == "NET_IP" else config.get("Settings", "CVN_IP")
    streamcvIp = netIp if config.get("Settings", "STREAMCV_IP") == "NET_IP" else config.get("Settings", "STREAMCV_IP")
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
    dridIp = netIp if config.get("Settings", "DRID_IP") == "NET_IP" else config.get("Settings", "DRID_IP")
    mediamtxIp = netIp if config.get("Settings", "MEDIAMTX_IP") == "NET_IP" else config.get("Settings", "MEDIAMTX_IP")

    keycloakIp = netIp if config.get("Keycloak", "KEYCLOAK_IP") == "NET_IP" else config.get("Keycloak", "KEYCLOAK_IP")
    kafkaLocalIp = netIp if config.get("Settings", "KAFKA_LOCAL_IP") == "NET_IP" else config.get("Settings", "KAFKA_LOCAL_IP")
    
    dbWhitelist = config.get("Settings", "DB_WHITELIST")


    #################################
    ### CREATE KEYCLOAK .env FILE ###
    #################################

    if shouldCreateKeycloakEnvFile:
        keycloakDbPassword = ''.join(random.choices(string.ascii_letters + string.digits, k=16))
        keycloakAdminPassword = ''.join(random.choices(string.ascii_letters + string.digits, k=16))
        keycloakClientSecret = ''.join(random.choices(string.ascii_letters + string.digits, k=32))
        envData = {
            "KEYCLOAK_DB_VENDOR": config.get("Keycloak", "KEYCLOAK_DB_VENDOR"),
            "KEYCLOAK_DB_ADDR": config.get("Keycloak", "KEYCLOAK_DB_ADDR"),
            "KEYCLOAK_DB_DATABASE": config.get("Keycloak", "KEYCLOAK_DB_DATABASE"),
            "KEYCLOAK_DB_USER": config.get("Keycloak", "KEYCLOAK_DB_USER"),
            "KEYCLOAK_DB_PASSWORD": keycloakDbPassword,
            "POSTGRES_USER": config.get("Keycloak", "KEYCLOAK_DB_USER"),
            "POSTGRES_PASSWORD": keycloakDbPassword,
            "POSTGRES_DB": config.get("Keycloak", "KEYCLOAK_DB_DATABASE"),
            "KEYCLOAK_ADMIN": config.get("Keycloak", "KEYCLOAK_ADMIN"),
            "KEYCLOAK_ADMIN_PASSWORD": keycloakAdminPassword,
            "KEYCLOAK_DJANGO_CLIENT_ID": config.get("Keycloak", "KEYCLOAK_DJANGO_CLIENT_ID"),
            "KEYCLOAK_DJANGO_CLIENT_SECRET": keycloakClientSecret,
        }
        with open(keycloakEnvFilePath, 'w') as f:
            for key, value in envData.items():
                f.write(f"{key}={value}\n")
        

    ########################
    ### CREATE .env FILE ###
    ########################

    if shouldCreateEnvFile:
        load_dotenv(keycloakEnvFilePath) # load the keycloak .env file to get the client secret
        envData = {
            # VERSION
            "VERSION": config.get("About", "VERSION"),
            # HOSTS
            "ALG_HOST": algHost,
            "CCD_HOST": ccdHost,
            "CV_HOST": cvHost,
            "CVN_HOST": cvnHost,
            "STREAMCV_HOST": streamcvHost,
            "DB_HOST": dbHost,
            "LSC_HOST": lscHost,
            "GEO_HOST": geoHost,
            "MAV_HOST": mavHost,
            "NGINX_HOST": nginxHost,
            "ODM_HOST": odmHost,
            "ROS_HOST": rosHost,
            "RTMP_HOST": rtmpHost,
            "WEB_HOST": webHost,
            "WS_HOST": wsHost,
            "WSI_HOST": wsiHost,
            "WSM_HOST": wsmHost,
            "KC_HOST": kcHost,
            "DRID_HOST": dridHost,
            # IP configuration
            "SUBNET": subnet,
            "GATEWAY": gateway,
            "DB_WHITELIST":dbWhitelist,
            # IP ADDRESS
            "ALG_IP": algIp,
            "CCD_IP": ccdIp,
            "CV_IP": cvIp,
            "CVN_IP": cvnIp,
            "STREAMCV_IP": streamcvIp,
            "DB_IP": dbIp,
            "GEO_IP": geoIp,
            "LSC_IP": lscIp,
            "MAV_IP": mavIp,
            "NGINX_IP": nginxIp,
            "ODM_IP": odmIp,
            "ROS_IP": rosIp,
            "RTMP_IP": rtmpIp,
            "WEB_IP": webIp,
            "WS_IP": wsIp,
            "WSI_IP": wsiIp,
            "WSM_IP": wsmIp,
            "KC_IP": kcIp,
            "DRID_IP": dridIp,
            "MEDIAMTX_IP": mediamtxIp,
            # PORTS
            "ALG_API_PORT": config.get("Settings", "ALG_API_PORT"),
            "CV_API_PORT": config.get("Settings", "CV_API_PORT"),
            "CVN_API_PORT": config.get("Settings", "CVN_API_PORT"),
            "STREAMCV_PORT": config.get("Settings", "STREAMCV_PORT"),
            "DB_PORT": config.get("Settings", "DB_PORT"),
            "GEO_PORT": config.get("Settings", "GEO_PORT"),
            "IUS_PORT": config.get("Settings", "IUS_PORT"),
            "LSC_API_PORT": config.get("Settings", "LSC_API_PORT"),
            "MAV_API_PORT": config.get("Settings", "MAV_API_PORT"),
            "NGINX_PORT": config.get("Settings", "NGINX_PORT"),
            "ROS_API_PORT": config.get("Settings", "ROS_API_PORT"),
            "ODM_PORT": config.get("Settings", "ODM_PORT"),
            "RTMP_PORT": config.get("Settings", "RTMP_PORT"),
            "WEB_PORT": config.get("Settings", "WEB_PORT"),
            "WEB_WS_PORT": config.get("Settings", "WEB_WS_PORT"),
            "WS_PORT": config.get("Settings", "WS_PORT"),
            "WSI_PORT": config.get("Settings", "WSI_PORT"),
            "WSM_PORT": config.get("Settings", "WSM_PORT"),
            "KC_API_PORT": config.get("Settings", "KC_API_PORT"),
            "DRID_API_PORT": config.get("Settings", "DRID_API_PORT"),
            "KEYCLOAK_PORT": config.get("Settings", "KEYCLOAK_PORT"),
            "MEDIAMTX_PORT": config.get("Settings", "MEDIAMTX_PORT"),

            #DJANGO
            "DB_ENGINE": config.get("Settings", "DB_ENGINE"),
            # DATABASE
            "DB_DATABASE": config.get("Settings", "DB_DATABASE"),
            # DATABASE CONNECTIONS
            "ALG_DB_CONNECTION_POOLS": config.get("Settings", "ALG_DB_CONNECTION_POOLS"),
            "CV_DB_CONNECTION_POOLS": config.get("Settings", "CV_DB_CONNECTION_POOLS"),
            "CVN_DB_CONNECTION_POOLS": config.get("Settings", "CVN_DB_CONNECTION_POOLS"),
            "LSC_DB_CONNECTION_POOLS": config.get("Settings", "LSC_DB_CONNECTION_POOLS"),
            "MAV_DB_CONNECTION_POOLS": config.get("Settings", "MAV_DB_CONNECTION_POOLS"),
            "ROS_DB_CONNECTION_POOLS": config.get("Settings", "ROS_DB_CONNECTION_POOLS"),
            "KC_DB_CONNECTION_POOLS": config.get("Settings", "KC_DB_CONNECTION_POOLS"),
            "DRID_DB_CONNECTION_POOLS": config.get("Settings", "DRID_DB_CONNECTION_POOLS"),
            # CUSTOM
            "NET_IP": netIp,
            "DB_USER": databaseUsername,
            "DB_PASSWORD": databasePassword,
            "ADMIN_USER": webAdminUsername,
            "ADMIN_PASSWORD": webAdminPassword,

            "NVIDIA_AVAILABLE": nvidiaAvailable,
            "DEBUG": debugMode,
            "OPEN_NETWORK": openNetworkMode,
            "DJANGO_ALLOWED_HOSTS": "* localhost 127.0.0.1",
            "SECRET_KEY": "django-insecure-7u@(b_01go-msdw=*smjl0(4+02scu=&)m-(*6x%9+wp4tik$^",
            "TZ": timezone,
            "STREAM_CAPTURE_FPS": streamFps,
            "COMPUTER_VISION_FPS": cvFps,
            "VIDEO_AND_CV_REMOTE": remoteVideoAndCv,
            "VIDEO_AND_CV_SERVER":"null",
            "VIDEO_AND_CV_PORT":"null",
            "LEGACY_LSC": config.get("Settings", "LEGACY_LSC"),

            # KAFKA
            "KAFKA_ACTIVE": config.get("Settings", "KAFKA_ACTIVE"),
            "KAFKA_LOCAL": config.get("Settings", "KAFKA_LOCAL"),
            "KAFKA_LOCAL_IP": kafkaLocalIp,
            "KAFKA_IP": config.get("Settings", "KAFKA_IP"),
            "KAFKA_PORT": config.get("Settings", "KAFKA_PORT"),
            "KAFKA_GROUP_ID": config.get("Settings", "KAFKA_GROUP_ID"),
            "KAFKA_TELEMETRY_TOPIC": config.get("Settings", "KAFKA_TELEMETRY_TOPIC"),
            "KAFKA_PATH_PLANNING_OUTPUT_TOPIC": config.get("Settings", "KAFKA_PATH_PLANNING_OUTPUT_TOPIC"), 
            "KAFKA_OBJECT_DETECTION_TOPIC": config.get("Settings", "KAFKA_OBJECT_DETECTION_TOPIC"),
            "KAFKA_CRISIS_CLASSIFICATION_TOPIC": config.get("Settings", "KAFKA_CRISIS_CLASSIFICATION_TOPIC"),

            "KAFKA_TELEMETRY_GROUP_ID": config.get("Settings", "KAFKA_TELEMETRY_GROUP_ID"),
            "KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID": config.get("Settings", "KAFKA_PATH_PLANNING_OUTPUT_GROUP_ID"),
            "KAFKA_CRISIS_CLASSIFICATION_GROUP_ID": config.get("Settings", "KAFKA_CRISIS_CLASSIFICATION_GROUP_ID"),
            "KAFKA_OBJECT_DETECTION_GROUP_ID": config.get("Settings", "KAFKA_OBJECT_DETECTION_GROUP_ID"),

            # KEYCLOAK
            "KEYCLOAK_ACTIVE": config.get("Keycloak", "KEYCLOAK_ACTIVE"),
            "KEYCLOAK_IP": keycloakIp,
            "KEYCLOAK_DJANGO_CLIENT_ID": config.get("Keycloak", "KEYCLOAK_DJANGO_CLIENT_ID"),
            "KEYCLOAK_DJANGO_CLIENT_SECRET": os.getenv("KEYCLOAK_DJANGO_CLIENT_SECRET", ""),
            "KEYCLOAK_ADMIN": os.getenv("KEYCLOAK_ADMIN", ""),
            "KEYCLOAK_ADMIN_PASSWORD": os.getenv("KEYCLOAK_ADMIN_PASSWORD", ""),

            "ENABLE_WEATHER_FETCH": config.get("Settings", "ENABLE_WEATHER_FETCH"),
            "MTX_USER": config.get("Settings", "MTX_USER"),
            "MTX_PASS": config.get("Settings", "MTX_PASS"),


        }
        with open(envFilePath, 'w') as f:
            for key, value in envData.items():
                f.write(f"{key}={value}\n")


def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory


def getParentDirectory():
    parentDdirectory = os.path.dirname(getCurrentDirectory())
    return parentDdirectory


def envFileExists(_envFilePath): 
    return os.path.isfile(_envFilePath)


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
    

def readConfigFile(_configFile):
    c = configparser.ConfigParser()
    c.read(_configFile)
    return c



configPath = os.path.join(getParentDirectory(), 'config.ini')
config = readConfigFile(configPath)
envFilePath = os.path.join(getParentDirectory(), '.env')
keycloakEnvFilePath = os.path.join(getParentDirectory(), 'keycloak', '.env')


if __name__ == "__main__":
    if not envFileExists(envFilePath):
        print("Creating environment files...")
        createEnvFiles()
        print("Environment files created successfully.")
    else:
        print("Environment files already exist. No changes made.")