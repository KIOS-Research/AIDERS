import serial
import signal
import sys
import json
import requests
from time import gmtime, strftime
import time
from aiders import models,serializers

lat = 35.007696676570056
lon =  33.2726922542

def receiveLocations( port):
    global flag
    global lat
    global lon
    flag = True
    baud = 4800
    firstTime = True
    try:
        connection = serial.Serial(port, baud, timeout=1)
    except serial.serialutil.SerialException as e:
        print(e)
        print("No lora receiver station plugged on this PC. Waiting...")
        connection = keepRetrying(port, baud)
    while flag:
        try:

            data = connection.readline()
            data = data.decode('UTF-8')
            data = data.replace('"{','{"')











        except serial.serialutil.SerialException as e:
            print(e)
            print("Seems like the lora receiver station was disconnected! ")
            print("Waiting for lora receiver station to reconnect...")
            connection = keepRetrying(port, baud)

        # print(data)
        try:
            # import pdb;pdb.set_trace()
            data = json.loads(data)

            if (firstTime):
                prev = lat
                data["coordinates"]["lat"] = lat
                firstTime = False
            else:
                data["coordinates"]["lat"] = prev
                prev = prev + 0.0002
            data["coordinates"]["lon"] = lon


            print("LORA DATA:", data)




            tagName = data.get('tagName')
            uptime = data.get('waketime')
            lat = data.get('coordinates').get('lat')
            lon = data.get('coordinates').get('lon')

            # data = json.dumps(data)
            try:
                obj = models.LoraTransmitter.objects.get(tagName=tagName)
                # print("OBJECT EXISTS!!!: ,", obj.__dict__)
                # print("OBJECT tag name: ", obj.tagName)
                obj.upTime = uptime
                obj.save()
            except models.LoraTransmitter.DoesNotExist:  #This lora device does not exist in the database yet. Let's add it
                print("OBJECT DOES NOT EXIST I WILL CREATE IT!!")
                serializer = serializers.LoraTransmitterSerializer(data={"tagName":tagName, "upTime":uptime})

                if serializer.is_valid():
                    obj = serializer.save()
                else:
                    print("HERE1")
                    print("ERRORS: ", serializer.errors)

            loraTransmitterObj = {
                "loraTransmitter": obj.id  ,
                "lat": lat,
                "lon":lon
            }

            serializer = serializers.LoraTransmitterLocationSerializer(data=loraTransmitterObj)

            if serializer.is_valid():
                serializer.save()
            else:
                print("here2")
                print("ERRORS: ", serializer.errors)
        except ValueError as e:
            pass

    connection.close()
def keepRetrying(port,baud):
    connection = None
    while(1):
        time.sleep(1)
        try:
            connection = serial.Serial(port, baud, timeout=1)
            connection.flush()
            print("Lora station reconnected!")
            break
        except Exception as e:
#             print("retrying..")
            # print("Exception occured!")
            # print(e)

            continue

    return connection
def main():
    lora_stations_api_url = None
    if (len(sys.argv) < 3):
        print("Make sure you provide the API route for lora stations")
        exit()
    else:
        lora_stations_api_url = sys.argv[1]
        device_symlink_port_path = sys.argv[2]

    print("Lora Location receiver script started! ")
    receiveLocations(device_symlink_port_path)
def postToAPI(payload, api_url):
    url = api_url + "/lora_stations"
    headers = {
        'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)
    # print(response)
if __name__ == '__main__':
    main()
