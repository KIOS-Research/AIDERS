#!/usr/bin/env python
import json
import math
import socket
import struct
import threading
from decimal import *

import geopy
import requests
import rospy
from aiders.models import (BuildMapImage, BuildMapSession, Drone, Operation,
                           User)
from django.conf import settings
from django.contrib.gis.geos import Point
from geopy.distance import geodesic
from kios.msg import BuildMap, PicServer
from .build_map_request_handler import adjustRate
from logic.Constants import Constants


class Photo:
    def set_photo_atrs(self,name,p1lat,p1lon,p2lat,p2lon,p3lat,p3lon,p4lat,p4lon, centerlat, centerlon, bearing, alt):
        self.name = name
        self.p1lat = p1lat
        self.p1lon = p1lon
        self.p2lat = p2lat
        self.p2lon = p2lon
        self.p3lat = p3lat
        self.p3lon = p3lon
        self.p4lat = p4lat
        self.p4lon = p4lon
        self.alt = alt
        self.bearing = bearing
        self.centerlat = centerlat
        self.centerlon = centerlon

    def get_photo_id(self):
        return self.name
import os

rateHz = 150
def rosCallback(data, args):
    lat = data.latitude
    lon = data.longitude
    h = data.altitude
    b = data.heading
    photo_obj = args[0]
    drone_name = args[1]
    droneModel = args[2]
    BuildMapSessionObject = args[3]

    # print('lat: ' + str(lat))
    # print('lon: ' + str(lon))
    # print('h: ' + str(h))
    # print('b: ' + str(b))
    # print('photo_id: ' + str(data.photoid))
    photoid = data.photoid
    destinations = calcPoints(lat, lon, b, h, photoid, droneModel)

    createPhotoObject(destinations,photoid, photo_obj, lat, lon, b, h)
    appendPhotoToDjango(photo_obj, drone_name, BuildMapSessionObject)

def appendPhotoToDjango(photoObj, drone_name, BuildMapSessionObject):
    image = BuildMapImage.objects.create(
        path = str(drone_name)+'/'+str(photoObj.name)+'.jpeg',
        top_left = Point(photoObj.p1lon, photoObj.p1lat),
        top_right = Point(photoObj.p2lon, photoObj.p2lat),
        bottom_left =Point(photoObj.p3lon, photoObj.p3lat),
        bottom_right =Point(photoObj.p4lon, photoObj.p4lat),
        centre =Point(photoObj.centerlon, photoObj.centerlat),
        altitude = Decimal(photoObj.alt),
        bearing = Decimal(photoObj.bearing),
    )
    BuildMapSessionObject.images.add(image)
    print('Database BuildMapImage is created')
    print('Drone ' +drone_name + ' saved data')
    # operation_instance.drones_to_operate.set(drone_allow_list)
    # writer.writerow([photoObj.name, photoObj.p1lat, photoObj.p1lon, photoObj.p2lat, photoObj.p2lon, photoObj.p3lat ,photoObj.p3lon ,photoObj.p4lat, photoObj.p4lon, photoObj.alt, photoObj.bearing, photoObj.centerlat, photoObj.centerlon])


def createPhotoObject(destinations, photoid, photo_obj, centerLat, centerLon, bearing, altitude):
    destination1 = destinations[0]
    destination2 = destinations[1]
    destination3 = destinations[2]
    destination4 = destinations[3]

    photo_obj.set_photo_atrs(
                                name=photoid,
                                p1lat=destination1.latitude, #upper right
                                p2lat= destination2.latitude, #lower right
                                p3lat= destination4.latituphoto_source_idde, #lower left
                                p4lat= destination3.latitude, #upper left
                                p1lon= destination1.longitude,
                                p2lon= destination2.longitude,
                                p3lon= destination4.longitude,
                                p4lon=destination3.longitude,
                                alt=altitude,
                                bearing=bearing,
                                centerlat = centerLat,
                                centerlon = centerLon
                             )

    return photo_obj


def get_current_file_name_from_api(buildmap_current_txt_file_name_api_url):
    url = buildmap_current_txt_file_name_api_url
    response = requests.request("GET", url)
    return response.json()["buildmap_csv_file_local_path"]


def get_current_subfolder_name_for_this_drone_from_api(buildmap_current_subfolder_name_api_url):
    url = buildmap_current_subfolder_name_api_url
    response = requests.request("GET", url)
    return response.json()["buildmap_photo_folder_local_path"]

def listener(thread, dji_name="mavic_blue", ip='', port=0, sock=None, network_ip=None, drone_model=None, username='user', operation="operation"):
    rate = rospy.Rate(rateHz)
    photo_obj = Photo()
    BuildMapSessionObject=BuildMapSession.objects.create(user=User.objects.get(username=username), operation=Operation.objects.get(operation_name = operation), drone=Drone.objects.get(drone_name = dji_name))
    subscriber = rospy.Subscriber("/" + dji_name + "/BuildMapResponse", BuildMap, rosCallback, (photo_obj, dji_name, drone_model, BuildMapSessionObject))
    rate.sleep()

    publisher = rospy.Publisher("/" + dji_name + "/PicServerSetup", PicServer, queue_size=10)
    rate.sleep()
    picMessage = PicServer()
    # picMessage.serverIP = ip
    picMessage.serverIP = network_ip
    picMessage.serverPort = port
    publisher.publish(picMessage)
    drone=Drone.objects.get(drone_name = dji_name)
    drone.build_map_activated = True
    drone.save()
    print("\nPhoto listener script started for drone {}!".format(dji_name))
    print("\nPublished ip and port: {}\n".format(str(picMessage)))
    sock.listen(1000)

    while not rospy.is_shutdown():
        client, addr = sock.accept()
        print('got connected from', addr)
        print('Drone:', dji_name)
        buf = b""
        while len(buf) < 4:
            buf += client.recv(4 - len(buf))
        size = struct.unpack('!i', buf)
        print("receiving %s bytes" % size)
        photo_name = photo_obj.get_photo_id()
        imgData = bytearray(b'')
        while True:
            tempData = bytearray(client.recv(1024))
            imgData.extend(tempData)
            if not tempData:
                break
        #IMAGE DATA RECEIVED!
        try:
            os.mkdir(os.path.join(settings.MEDIA_ROOT, str(dji_name)))
        except:
            pass
        full_filename = os.path.join(settings.MEDIA_ROOT, str(dji_name), str(photo_name)+'.jpeg')
        print(full_filename)
        with open(full_filename, 'wb') as img:
            img.write(imgData)
        if thread.stopped():
            client.close()
            break
        rate.sleep()
    client.close()
    
def high_accuracy_image_center(lat, long, altitude, pitch, roll, bearing):
    distance_pitch = altitude * math.tan(pitch*math.pi/180)#lat
    distance_roll = altitude * math.tan(roll*math.pi/180)#long
    destination_pitch=geodesic(kilometers=distance_pitch/1000).destination((0, 0), bearing+0)
    destination_roll=geodesic(kilometers=distance_roll/1000).destination((0, 0), bearing+270)
    newLat=lat+destination_pitch.latitude+destination_roll.latitude
    newLong=long+destination_pitch.longitude+destination_roll.longitude
    return(newLat, newLong)

def calcPoints(lat1, lon1, bearing, h, photoid, droneModel, cameraModel):
    #reference https://www.propelleraero.com/blog/ground-sample-distance-gsd-calculate-drone-data/

    bearing = bearing - 90
    if cameraModel == Constants.Mavic2Camera:
        fovH = 68.064344 * math.pi/180 # The angle on the pyramid from the camera to the ground
        fovV = 40.045496 * math.pi/180

        imW = 1920  # To mikos (width)  se PIXELS tis vasis tis piramidas
        imH = 1080
    elif cameraModel == Constants.H20TWideCamera:
        fovH = 68.857347 * math.pi/180 # The angle on the pyramid from the camera to the ground
        fovV = 41.716547 * math.pi/180

        imW = 608  # To mikos (width)  se PIXELS tis vasis tis piramidas
        imH = 342
    else:
        fovH = 68.064344 * math.pi/180 # The angle on the pyramid from the camera to the ground
        fovV = 40.045496 * math.pi/180

        imW = 1920  # To mikos (width)  se PIXELS tis vasis tis piramidas
        imH = 1080
    # fovH = 68.064344 * math.pi/180 # The angle on the pyramid from the camera to the ground
    # fovV = 40.045496 * math.pi/180

    # if (droneModel == MAVIC_2_ENTERPRISE ):
    #     fovH = 68.064344 * math.pi / 180  # The angle on the pyramid from the camera to the ground
    #     fovV = 40.045496 * math.pi / 180
    #
    #     imW = 1920  # To mikos (width)  se PIXELS tis vasis tis piramidas
    #     imH = 1080




    """
    H20 WIDE PARAMETERS (photo mode):
        FOV Degrees:
            FOV_H	69.357647
            FOV_V	53.761636
        RESOLUTION:
            WIDTH: 4056
            destination1HEIGHT: 3040
    
    H20 ZOOM PARAMETERS (photo mode):
        FOV Degrees:
            FOV_H	35.595534
            FOV_V	27.320463
        RESOLUTION:
            WIDTH: 5184
            HEIGHT: 3888
            
  
    MAVIC2E (photo mode): 
        FOV Degrees:
            FOV_H	68.064344
            FOV_V	40.045496
        
        RESOLUTION:
            WIDTH: 1920 (??)
            HEIGHT: 1080 (??)
    """


    dw = 2 * h * abs(math.tan(fovH/2)) #To width se metra tis vasis tis piramidas pou sximatizete
    dh = 2 * h * abs(math.tan(fovV/2)) #To height se metra tis vasis tis piramidas pou sximatizete

    d = math.sqrt(dw ** 2 + dh ** 2)     # Pithagorio theorima gia na vroume tin diagonio tis photografias se metra
    d = d /1000     #converting ti diagonio se km
    d = d/2     #pianoume ti misi diagonio


    a = 90 - math.atan(imW/imH) * (180/math.pi) #i gonia tou pou vlepei to drone me ti gonia tis photografias

    origin = geopy.Point(lat1, lon1) #The center of the image in lat long form. This is the starting point

    destination1 = geodesic(kilometers=d).destination(origin, bearing + a) # Panw deksia Dias tou tin apostasi kai tin gonia se sxesi me ton vorra, kai sou dinei to simio tou destination se lat long
    destination2 = geodesic(kilometers=d).destination(origin, bearing + 180 + a) #Katw aristera
    destination3 = geodesic(kilometers=d).destination(origin, bearing - a) #Panw aristera
    destination4 = geodesic(kilometers=d).destination(origin, bearing - 180 - a) #katw deksia

    # print("[", destination1.longitude, ",", destination1.latitude, "],")
    # print("[", destination4.longitude, ",", destination4.latitude, "],")
    # print("[", destination2.longitude, ",", destination2.latitude, "],")
    # print("[", destination3.longitude, ",", destination3.latitude, "],")

    return [destination1, destination2, destination3, destination4]


def empty_txt_file(txt_path):
    if  os.path.exists(txt_path):
        open(txt_path, 'w').close()


def create_file_if_not_exists(txt_path):
    if (not os.path.exists(txt_path)):
        with open(txt_path, 'w') as fp:
            pass

def post_port_to_api(drone_id,port):

    url = "http://localhost:5000/buildmap/port/" + drone_id
    payload = {
        "port": port
    }
    payload = json.dumps(payload)
    headers = {
        'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)

    print(response.text)

def tryPort(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = False
    try:
        sock.bind((ip, port))
        result = True
    except:
        print("Port is in use")
    sock.close()
    return result

def main(thread_name, dji_name, ip, port, user, operation):


    
    for thread in threading.enumerate():
        if thread.name == thread_name:
            try:
                print("Image georeference script has started on ip: " + ip + " and port: " + str(port) + " for Drone: " + dji_name)
                address = (ip, port)  # ip of this machine. MUltiple ports if multiple drones do this
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.settimeout(10000)
                s.bind(address)
                rospy.init_node('dji_input', anonymous=True)
                private_param = rospy.get_param('~dji_name', dji_name)
                print("In georeference", dji_name)
                listener(thread, private_param, ip, port, s, os.environ['NET_IP'], None, user, operation)

            except rospy.ROSInterruptException:
                pass

if __name__ == '__main__':
    main()