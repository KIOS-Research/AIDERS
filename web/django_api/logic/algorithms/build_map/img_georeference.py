#!/usr/bin/env python
import math
from decimal import *

import geopy
from geopy.distance import geodesic
from logic.Constants import Constants


def calcPoints(lat1, lon1, bearing, h, photoid, droneModel, cameraModel):
    # reference https://www.propelleraero.com/blog/ground-sample-distance-gsd-calculate-drone-data/

    bearing = bearing - 90
    if cameraModel == Constants.Mavic2Camera:
        fovH = (
            68.064344 * math.pi / 180
        )  # The angle on the pyramid from the camera to the ground
        fovV = 40.045496 * math.pi / 180

        imW = 1920  # To mikos (width)  se PIXELS tis vasis tis piramidas
        imH = 1080
    elif cameraModel == Constants.H20TWideCamera:
        fovH = (
            68.857347 * math.pi / 180
        )  # The angle on the pyramid from the camera to the ground
        fovV = 41.716547 * math.pi / 180

        imW = 608  # To mikos (width)  se PIXELS tis vasis tis piramidas
        imH = 342
    else:
        fovH = (
            68.064344 * math.pi / 180
        )  # The angle on the pyramid from the camera to the ground
        fovV = 40.045496 * math.pi / 180

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

    dw = (
        2 * h * abs(math.tan(fovH / 2))
    )  # To width se metra tis vasis tis piramidas pou sximatizete
    dh = (
        2 * h * abs(math.tan(fovV / 2))
    )  # To height se metra tis vasis tis piramidas pou sximatizete

    d = math.sqrt(
        dw**2 + dh**2
    )  # Pithagorio theorima gia na vroume tin diagonio tis photografias se metra
    d = d / 1000  # converting ti diagonio se km
    d = d / 2  # pianoume ti misi diagonio

    a = 90 - math.atan(imW / imH) * (
        180 / math.pi
    )  # i gonia tou pou vlepei to drone me ti gonia tis photografias

    origin = geopy.Point(
        lat1, lon1
    )  # The center of the image in lat long form. This is the starting point

    destination1 = geodesic(kilometers=d).destination(
        origin, bearing + a
    )  # Panw deksia Dias tou tin apostasi kai tin gonia se sxesi me ton vorra, kai sou dinei to simio tou destination se lat long
    destination2 = geodesic(kilometers=d).destination(
        origin, bearing + 180 + a
    )  # Katw aristera
    destination3 = geodesic(kilometers=d).destination(
        origin, bearing - a
    )  # Panw aristera
    destination4 = geodesic(kilometers=d).destination(
        origin, bearing - 180 - a
    )  # katw deksia

    # print("[", destination1.longitude, ",", destination1.latitude, "],")
    # print("[", destination4.longitude, ",", destination4.latitude, "],")
    # print("[", destination2.longitude, ",", destination2.latitude, "],")
    # print("[", destination3.longitude, ",", destination3.latitude, "],")

    return [destination1, destination2, destination3, destination4]
