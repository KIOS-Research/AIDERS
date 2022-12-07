import sys
from distutils.command.config import LANG_EXT

import psutil
import rospy
from aiders import models, serializers, views
from django.contrib.auth import get_user_model

from .algorithms.fire_prediction import firePrediction
from .algorithms.mission import mission_request_handler
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
