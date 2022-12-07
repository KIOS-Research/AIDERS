
import json
import logging
import os
import sys

import numpy as np
import pandas as pd
import rospy
# import utm
from aiders import models, views
from std_msgs.msg import String

logger = logging.getLogger(__name__)


def rosCallback(data, args):

    dji_name = args[0]
    lidar_session = args[1]
    data = data.data
    json_data = json.loads(str(data))
    pointCloudData = json_data["PointCloud:"]
    views.LidarPointsAPIView.save_point_in_db(
        pointCloudData, dji_name, lidar_session)


'''
Setup listeners/subscribers
Main loop!
'''


def listener(dji_name="kios_mavic1g", lidar_session=None):
    subscriber = rospy.Subscriber(
        "/" + dji_name + "/PointCloud", String, rosCallback, (dji_name, lidar_session))
    rospy.spin()


def PublishMessage(drone_name, data):
    publisher = rospy.Publisher(
        "/" + drone_name + "/StartOrStopPointCloud", String, queue_size=10)
    t = String()
    t.data = data
    publisher.publish(t)


# def lidar_points_to_long_lat():
#     print('')

    # pos_x, pos_y, pos_z = 1, 0, 100
    # lat_origin, lon_origin, alt_origin = 35, 33, 100
    # x_degrees = 0
    # y_degrees = 0
    # z_degrees = 0
    # import math
    # new_pointx = lat_origin-(pos_x)
    # new_pointy = lon_origin-(pos_y)
    # new_pointz = alt_origin-(pos_z)
    # print(new_pointx, new_pointy, new_pointz)
    # Rx_l2u = np.matrix(
    #     [[1, 0, 0],
    #         [0, np.cos(x_degrees), -np.sin(x_degrees)],
    #         [0, np.sin(x_degrees), np.cos(x_degrees)]]
    # )
    # Ry_l2u = np.matrix(
    #     [[np.cos(y_degrees), 0, np.sin(y_degrees)],
    #         [0, 1, 0],
    #         [-np.sin(y_degrees), 0, np.cos(y_degrees)]]
    # )
    # Rz_l2u = np.matrix(
    #     [[np.cos(z_degrees), -np.sin(z_degrees), 0],
    #         [np.sin(z_degrees),  np.cos(z_degrees), 0],
    #         [0, 0, 1]]
    # )
    # origin_utm = utm.from_latlon(lat_origin, lon_origin)
    # xyz_pts_lidar = np.concatenate(([pos_x], [pos_y], [pos_z]), axis=0)

    # + np.matrix([[origin_utm[0]], [origin_utm[1]]])

    # xy_pts_lidar = np.concatenate(([pos_x], [pos_y]), axis=0)

    # print("Shape of xy_lidar pts is: ", xy_pts_lidar.shape)

    # # Add heading angle of the LiDAR
    # heading_lidar = np.radians(90)    # North is +90 deg, East is 0 deg
    # # Add gimbal angle of the LiDAR

    # # Rotation matrix - LIDAR to UTM frame:
    # # R_l2u = np.matrix([[np.cos(heading_lidar), -np.sin(heading_lidar)],
    # #                    [np.sin(heading_lidar),  np.cos(heading_lidar)]])
    # R_l2u = np.matrix([[np.cos(heading_lidar), -np.sin(heading_lidar)],
    #                    [np.sin(heading_lidar),  np.cos(heading_lidar)]])

    # R_l2u = np.matrix([[np.cos(heading_lidar), -np.sin(heading_lidar)],
    #                    [np.sin(heading_lidar),  np.cos(heading_lidar)]])

    # x_degrees = 0
    # y_degrees = 0
    # z_degrees = 0
    # Rx_l2u = np.matrix(
    #     [[1, 0, 0],
    #      [0, np.cos(x_degrees), -np.sin(x_degrees)],
    #      [0, np.sin(x_degrees), np.cos(x_degrees)]]
    # )
    # Ry_l2u = np.matrix(
    #     [[np.cos(y_degrees), 0, np.sin(y_degrees)],
    #      [0, 1, 0],
    #      [-np.sin(y_degrees), 0, np.cos(y_degrees)]]
    # )
    # Rz_l2u = np.matrix(
    #     [[np.cos(z_degrees), -np.sin(z_degrees), 0],
    #      [np.sin(z_degrees),  np.cos(z_degrees), 0],
    #      [0, 0, 1]]
    # )

    # # Convert LIDAR origin to UTM frame:
    # origin_utm = utm.from_latlon(lat_origin, lon_origin)

    # print(" UTM origin is: ( ", origin_utm[0], " , ", origin_utm[1], " )")

    # # Rotate object points into UTM frame
    # xy_pts_utm = np.matmul(R_l2u, xy_pts_lidar) + \
    #     np.matrix([[origin_utm[0]], [origin_utm[1]]])

    # num_points = xy_pts_utm.shape[1]

    # # Convert XY pts from UTM to lat-lon:
    # lat_out = np.zeros(num_points)
    # lon_out = np.zeros(num_points)

    # for i in range(0, num_points):
    #     print(i)
    #     lat_lon = utm.to_latlon(
    #         xy_pts_utm[0, i], xy_pts_utm[1, i], origin_utm[2], origin_utm[3])
    #     lat_out[i] = lat_lon[0]
    #     lon_out[i] = lat_lon[1]
    # print(lat_out, lon_out)
