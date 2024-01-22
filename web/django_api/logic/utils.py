import sys
from distutils.command.config import LANG_EXT
import math
import threading
import psutil
from aiders import models, serializers, views
from django.contrib.auth import get_user_model

from .algorithms.fire_prediction import firePrediction
# from .algorithms.mission import mission_request_handler
from .algorithms.path_planning import pathPlanning
from .Constants import Constants


def signal_handler(sig, frame):
    _terminateProcesses(processNames=['python', 'rosout', 'StarterScript', 'python3'])
    sys.exit(0)

def add_path_to_sys(path):
    if path not in sys.path:
        sys.path.append(path)

def _terminateProcesses(processNames):
    for proc in psutil.process_iter():
        # check whether the process name matches
        if proc.name() in processNames:
            print("KILLED PYTHON PROCESS: " + str(proc.name()))
            proc.kill()


def get_ip_addr(request):
    if request.META.get('HTTP_X_FORWARDED_FOR'):
        ip = request.META.get('HTTP_X_FORWARDED_FOR')
    else:
        ip = request.META.get('REMOTE_ADDR')
    return ip


def get_client_ip(request):
    print("request: ", request)
    x_forwarded_for = request.META.get('HTTP_X_FORWARDED_FOR')
    if x_forwarded_for:
        ip = x_forwarded_for.split(',')[0]
    else:
        ip = request.META.get('REMOTE_ADDR')

    return ip

def handleAlgorithmExecution(operationPK, input, canBeLoadedOnMap, algorithmName, userPK):

    if algorithmName == models.Algorithm.CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM:
        batteries = input['batteries']
        xRange = input['xRange']
        yRange = input['yRange']
        currentPositions = input['currentPositions']
        currentAltitudes = input['currentAltitudes']
        lowerLeft = input['lowerLeft']
        upperLeft = input['upperLeft']
        lowerRight = input['lowerRight']
        upperRight = input['upperRight']
        selectedDroneIDs = input['selectedDroneIDs']
        paths = pathPlanning.getPaths(selectedDroneIDs,lowerLeft,upperLeft,
                                       lowerRight,upperRight,batteries,currentPositions,currentAltitudes,xRange,yRange)

        output = paths
        algorithmObj = {
            "algorithm_name": models.Algorithm.CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM,
            "output": output,
            "input": input,
            'user': userPK,
            'operation': operationPK,
            'canBeLoadedOnMap': canBeLoadedOnMap
        }

        # newAlgorithm = models.Algorithm(**algorithmObj)
        views.AlgorithmRetrieveView.save_algorithm_to_db(algorithmObj)

        # this is for SafeDrones collision detection algorithm
        if(len(paths) == 2):
            path1 = paths[0]["path"]
            path1.pop() # remove the last element to make the length of the two paths the same
            path2 = paths[1]["path"]
            # print(path1, flush=True)
            # print(path2, flush=True)
            # print(len(path1), flush=True)
            # print(len(path2), flush=True)

            import logic.algorithms.safe_drones.SafeDrones as SafeDrones
            eval = SafeDrones.SafeDrones()

            path1_tuples = [tuple(subarray) for subarray in path1]
            path2_tuples = [tuple(subarray) for subarray in path2]
            # print(path1_tuples, flush=True)
            # print(path2_tuples, flush=True)            
            danger_zone_risk, collision_zone_risk = eval.calculate_collision_risk(path1_tuples, path2_tuples, danger_threshold=15, collision_threshold=10)
            # print("danger_zone_risk", flush=True)
            # print(danger_zone_risk, flush=True)
            # print("collision_zone_risk", flush=True)
            # print(collision_zone_risk, flush=True)

            if(danger_zone_risk > 0.5):
                return [-1, danger_zone_risk, collision_zone_risk]
        for path in paths:
            thresholdAngle = 45
            path["path"]=getOnlyPathEdges(path["path"], thresholdAngle)
        return paths
    
    elif algorithmName == models.Algorithm.FIRE_PROPAGATION_ALGORITHM:
        fire_speed = input['fire_speed']
        fire_fronts = input['fire_fronts']
        wind_speed = input['wind_speed']
        wind_angle = input['wind_angle']
        time_steps = input['time_steps']['value']
        time_unit = input['time_steps']['units']
        lon = input['location']['lon']
        lat = input['location']['lat']
        time_intervals = input['time_intervals']

        firePredictions = firePrediction.getFirePrediction(fire_speed, fire_fronts, wind_speed, wind_angle, time_steps, time_unit, lon,lat , time_intervals )
        output = firePredictions
        algorithmObj = {
            "algorithm_name": models.Algorithm.FIRE_PROPAGATION_ALGORITHM,
            "output": output,
            "input": input,
            'user': userPK,
            'operation': operationPK,
            'canBeLoadedOnMap': canBeLoadedOnMap
        }
        views.AlgorithmRetrieveView.save_algorithm_to_db(algorithmObj)
        return output
    pass


def handleUserAction(operation, drone_name, user_name, action_json):
    pass
    # User = get_user_model()
    # print("ACTION: ",action_json)
    # action = action_json['action']
    # subaction = action_json['details']['subaction']
    # grid = action_json['details']['grid']
    # captureAndStoreImages = action_json['details']['captureAndStoreImages']
    # missionPath = action_json['details']['missionPath']
    # if action == User.MISSION_ACTION:
    #     print("OPERATION: " + str(operation))
    #     print("DRONE NAME: ", drone_name)
    #     print("USER NAME: ", user_name)
    #     print("subaction: ", subaction)
    #     print("grid: ", grid)
    #     print("captureAndStoreImages: ", captureAndStoreImages)
    #     print("missionpath: ", missionPath)
    #     mission_request_handler.publishMissionToRos(operation,drone_name,grid,captureAndStoreImages, missionPath,subaction)

def calculateAngle(_p1, _p2):
    # Calculate the angle between two points
    deltaX = _p2[0] - _p1[0]
    deltaY = _p2[1] - _p1[1]

    # Ensure that the angle is between 0 and 360 degrees
    angle = math.atan2(deltaY, deltaX) * (180 / math.pi)
    angle = (angle + 360) % 360
    return angle

def getOnlyPathEdges(_path, _thresholdAngle):
    filteredPoints = [_path[0]]  # Include the first point by default
    
    for i in range(1, len(_path) - 1):
        angle = calculateAngle(_path[i - 1][:2], _path[i][:2])
        next_angle = calculateAngle(_path[i][:2], _path[i + 1][:2])
        
        # Calculate the absolute difference between consecutive angles
        angle_difference = abs(next_angle - angle)
        
        # Check if the angle difference exceeds the threshold
        if angle_difference >= _thresholdAngle:
            filteredPoints.append(_path[i])
    filteredPoints.append(_path[-1])  # Include the last point by default
    
    return filteredPoints

# returns true if a thread with the same name is already running
def threadStarted(_threadName):
    return any(thread.name == _threadName for thread in threading.enumerate())

