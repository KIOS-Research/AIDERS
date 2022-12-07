import time

from geojson import Polygon, Feature, FeatureCollection
import math
import numpy as np
import pandas as pd
from random import randrange
from scipy import spatial
import sys

R_EARTH = 6371000.0


def xy_to_coords(pairs, srcPosition):
    coords = []

    for pair in pairs:
        new_latitude = srcPosition[0] + (pair[1] / R_EARTH) * (180 / math.pi)
        new_longitude = srcPosition[1] + (pair[0] / R_EARTH) * (180 / math.pi) / math.cos(new_latitude * math.pi / 180)
        coords.append((new_latitude, new_longitude))
    # print(coords)
    return coords


def create_polygon(pairs, timestep,firefronts , timeunit, featureid, windSpeed, fireSpeed):
    pairs.append(pairs[0])     # a polygon has to have as a last location, its first one to be enclosed
    polygon = Polygon([pairs])
    properties = {
        "ID" : featureid,
        "Time Step" : ("{} {}".format(timestep,timeunit)),
        "Fire Fronts" : firefronts,
        "Wind Speed (m/s)": windSpeed,
        "Fire Speed (m/s)" : fireSpeed
    }
    pol_feature  = Feature(geometry=polygon,properties=properties)
    return pol_feature


def print_arr(arr):
    names = ['x', 'y', 'z']
    index = pd.MultiIndex.from_product([range(s) for s in arr.shape], names=names)
    df = pd.DataFrame({'A': arr.flatten()}, index=index)['A']
    df = df.unstack(level='x').swaplevel().sort_index()
    df.columns = ['TS0', 'TS1', 'TS2']
    df.index.names = ['TIMESTEP', 'i']
    # print(df)


def generate_polygon(q_x_all, q_y_all, initial_ff, current_time_step, fire_fronts, time_unit, feature_id, windSpeed, fireSpeed, src_location):
    # print("Q_X_ALL: ")
    # print(q_x_all)
    q_points = [list(a) for a in zip(q_x_all, q_y_all)]
    q_x_convex = []
    q_y_convex = []
    q_all_convex = []
    hull = spatial.ConvexHull(q_points)
    # print(hull.vertices)
    for index in hull.vertices:
        q_x_convex.append(q_points[index][0])
        q_y_convex.append(q_points[index][1])
        q_all_convex.append((q_points[index][0], q_points[index][1]))

    lat_lon_convex = xy_to_coords(pairs=q_all_convex, srcPosition=src_location)
    fire_polygon = create_polygon(pairs=lat_lon_convex,timestep=current_time_step, firefronts=fire_fronts,
                                  timeunit=time_unit, featureid = feature_id, windSpeed=windSpeed,
                                  fireSpeed=fireSpeed)

    # plotWidget = pg.plot(
    #     title="time-stes: {} initial-ff: {} fire-fronts: {}".format(current_time_step, initial_ff, fire_fronts))
    # plotWidget2 = pg.plot(
    #     title="time-steps: {} initial-ff: {} fire-fronts: {}".format(current_time_step, initial_ff, fire_fronts))
    # plotWidget.plot(q_x_all, q_y_all, pen=None, symbol='o', symbolSize=7)
    # plotWidget2.plot(q_x_convex, q_y_convex, pen='g', symbol='x', symbolPen='g', symbolBrush=0.2, name='green')
    return fire_polygon


def predictFire(fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, lon, lat, time_intervals):
    location = [lon,lat]
    # R = 5  # Fire rate in m/s
    # time_steps = 1000
    # fire_fronts = 31
    dt = 1  # time step
    w_ = 0.01  # selected fixed threshold
    prev_fire_fronts = fire_fronts
    initial_ff = fire_fronts
    q = np.empty(
        (time_steps+1, fire_fronts , 2))  # Filling a numpy array with dimensions time_steps X firefronts*200 X 2
    q[:] = np.NaN

    q_x_all = []  # Initializing an array to store the  X coordinate for each firefront
    q_y_all = []
    interval_counter = 0
    all_polygons = []
    for i in range(0, time_steps+1):
        j = 0
        while j < fire_fronts:
            # print("size of second dim: " + str(np.size(q,1)))
            # print_arr(q)
            # print(q)
            # print("================================")
            cols = np.size(q,1)
            if (j>=cols):
                # print("WILL ADD COL: ")
                new_col = np.empty((time_steps+1,1,2))
                new_col[:] = np.NaN
                q = np.append(q, new_col,axis=1)
                # print(q)
            if (i == 0):
                q[i][j] = [0,0]  # For this inpout example, let's assume that the  fire starts at 100,100. That means  for time=0, all fire_fronts are at 100,100
                j += 1
                continue

            a = normrnd(wind_angle, 2)  # wind angle
            U = normrnd(wind_speed, 2)  # Wind rate in m/s
            if U < 0: U = 0  # If it happens to get wind speed < 0 make it 0. No negative speed exists
            LB = calc_LB(U)
            HB = calc_HB(LB)
            C = calc_C(HB, fire_speed)

            # The "NaN" value is the one that the array was initialized with. IT IS THE VALUE THAT SPECIFIES EMPTY CELL.
            # If the current Fire Front is a new one, that means there is no previous location of this fire front.
            # To determine the new location, get a random previous fire front location and do the formula
            # if(q[i - 1][j][0] == 0 and q[i - 1][j][1] == 0):
            if np.isnan(q[i - 1][j][0]) and np.isnan(q[i - 1][j][1]):
                prev_ff = randrange(prev_fire_fronts)
                q_x = q[i][prev_ff][0] + dt * C * math.cos(a)  # x coord of fire propagation
                q_y = q[i][prev_ff][1] + dt * C * math.sin(a)  # y coord of fire propagation
            else:
                q_x = q[i - 1][j][0] + dt * C * math.cos(a)  # x coord of fire propagation
                q_y = q[i - 1][j][1] + dt * C * math.sin(a)  # y coord of fire propagation
            q_x_all.append(q_x)
            q_y_all.append(q_y)
            q[i][j] = [q_x, q_y]
            j = j + 1

        d = int(uniformrnd(1, 3))  # Number of new fire-fronts
        w = uniformrnd(0, 1)  # A selected fixed threshhold
        prev_fire_fronts = fire_fronts
        if (w > w_): fire_fronts += d;  # To determine the number of new fire fronts
        # print("i= " + str(i))
        # print("interval counter= " + str(interval_counter))
        # print("time_intervals[interval_counter]= " + str(time_intervals[interval_counter]))
        if (interval_counter < len(time_intervals) and
                i == time_intervals[interval_counter]): #If we are at an interval the user requested AND the counter is still less than the total amount of the time intervals the user requested, generate the json for this interval.
            fire_polygon = generate_polygon(q_x_all, q_y_all, initial_ff, time_intervals[interval_counter], prev_fire_fronts, time_unit, interval_counter + 1, wind_speed, fire_speed, src_location=location)
            all_polygons.append(fire_polygon)
            interval_counter += 1


    return all_polygons


def getFirePrediction(fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, lon, lat, time_intervals):
    # time_intervals = list(time_intervals.split(","))
    time_intervals = list(map(int, time_intervals))
    all_fire_polygons = predictFire(fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, lon,lat,
                                    time_intervals)
    polygons_features = FeatureCollection([feature for feature in all_fire_polygons ])
    return polygons_features


def uniformrnd(mu, sigma):
    return np.random.uniform(mu, sigma, 1)[0]


def normrnd(mu, sigma):
    return np.random.normal(mu, sigma, 1)[0]


def calc_LB(U):
    LB = 0.936 * math.exp(0.2566 * U) + 0.461 * math.exp(-0.1548 * U) - 0.397
    return LB


def calc_HB(LB):
    par1 = LB + (LB ** 2 - 1) ** 0.5
    par2 = LB - (LB ** 2 - 1) ** 0.5
    HB = par1 / par2
    return HB


def calc_C(HB, R):
    C = (R - (R / HB)) / 2
    return C


if __name__ == '__main__':
    fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, location, time_intervals, lon, lat \
        = None, None, None, None, None, None, None, None, None, None
    if (len(sys.argv) < 9):
        print("\nPlease make sure you provide the following arguments:"
              "\n1)Fire Speed"
              "\n2)Number of Fire Fronts"
              "\n3)Wind Speed"
              "\n4)Wind Angle"
              "\n5)Time Steps"
              "\n6)Time Unit"
              "\n7)Location"
              )
        exit()
    else:
        fire_speed = float(sys.argv[1])
        fire_fronts = int(sys.argv[2])
        wind_speed = float(sys.argv[3])
        wind_angle = float(sys.argv[4])
        time_steps = int(sys.argv[5])
        time_unit = sys.argv[6]
        lon = float(sys.argv[7])
        lat = float(sys.argv[8])
        time_intervals = sys.argv[9]

        location = [lon, lat]  # lon lat

    # print(main(5,21,5,math.pi/8,1000, 's', [  33.377672 , 35.161117 ]))
    print(getFirePrediction(fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, lon,lat , time_intervals))
    # if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
    #     pg.QtGui.QApplication.exec_()
