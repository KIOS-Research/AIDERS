import json
import logging
import os
import sys
import time
from datetime import datetime

import requests
from aiders import models, serializers, views
from django.conf import settings
from django.contrib.auth import get_user_model
from pyodm import Node

logger = logging.getLogger(__name__)

TASK_COMPLETED = 40
TASK_RUNNING =  20

ORTHOPHOTO_TIF_FILE_NAME = 'odm_orthophoto.tif'
OBJECT_FILE_NAME = 'odm_textured_model_geo.obj'
OBJECT_INFO_FILE_NAME = 'object_info.json'

def get_tasks_status(prev_task_status):
    global NODE_ODM_API_URL
    url = NODE_ODM_API_URL + "/task/list"
    response = requests.request("GET", url, headers={'Content-Type': 'application/json'})
    task_uuids =  response.json()
    all_task_status = []
    just_completed_tasks = []
    for i,uuid in enumerate(task_uuids):
        uuid = uuid['uuid']
        url = NODE_ODM_API_URL + '/task/{}/info'.format(uuid)
        response = requests.request("GET", url, headers={'Content-Type': 'application/json'})
        status = response.json()['status']['code']
        just_completed = False
        if (len(prev_task_status) >= i+1):
            prevStatus = prev_task_status[i]['status']
            if (status == TASK_COMPLETED and prevStatus == TASK_RUNNING):
                print("TASK COMPLETED")
                just_completed_tasks.append({'uuid':uuid,'status':status})
                just_completed = True
        all_task_status.append({'uuid': uuid, 'status': status, 'just_completed': just_completed})
    return all_task_status
    # return {'all_task_status': all_task_status, 'just_completed_tasks': just_completed_tasks}
    # print(response.text)

def get_assets(uuid):
    url = '{}/task/{}/download/all.zip?token='.format(NODE_ODM_API_URL,uuid)
    response = requests.request("GET", url, headers={'Content-Type': 'application/json'})
    print(response)



def download_assets(task):
    global ALL_RESULT_FOLDERS_DIR
    # datetime object containing current date and time
    dt_string = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    result_folder_name  = str('object_3d_results_') + str(dt_string)
    results_folder_path = os.path.join(settings.OBJECT_3D_OUTPUTS_DIR, result_folder_name)
    # results_folder_url = os.path.join(result_folder_name,settings.OBJECT_3D_OUTPUTS_DIR)
    print("RESULTS FOLDER PATH: {}".format(results_folder_path))
    task.download_assets(results_folder_path)
    return  results_folder_path

def getPathContains(dir,s):
    for dirpath,_,filenames in os.walk(dir):
        for f in filenames:
            path = os.path.abspath(os.path.join(dirpath, f))
            if (s in path):
                return path
def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""
def updateObjectApiIP(dir,filename):
    '''
    Every time an object is generated, its URL is saved on a file.
    If user however decides to open a 3D object that was created on a different network,
    it won't load. For this reason, every time this script is started, it has to update the IP
    for object urls
    :param dir:
    :param filename:
    :return:
    '''
    global NET_IP
    for dirpath,_,filenames in os.walk(dir):
        for f in filenames:
            path = os.path.abspath(os.path.join(dirpath, f))
            if (filename in path):
                with open(path, 'r+') as f:
                    data = json.load(f)
                    data['uri'] = data['uri'].replace(find_between(data['uri'],'://',':'),NET_IP)  # replace the ip of the webserver link of the object with the IP of current network
                    f.seek(0)
                    json.dump(data, f, indent=4)
                    f.truncate()
def get_object_info(results_folder_path):

    orthophoto_path = getPathContains(results_folder_path, ORTHOPHOTO_TIF_FILE_NAME)
    orthophoto_url = os.path.relpath(orthophoto_path, settings.AIDERS_DIR)
    local_object_path = getPathContains(results_folder_path, OBJECT_FILE_NAME)
    object_url = os.path.relpath(local_object_path, settings.AIDERS_DIR)
    metadata = os.popen('gdalinfo -json {}'.format(orthophoto_path)).read()
    metadata = json.loads(metadata)
    upper_left_coord = metadata['wgs84Extent']['coordinates'][0][0]
    return {'upper_left_coord':upper_left_coord, 'object_url': object_url}


def start():
    global NET_IP
    prev_tasks_status = []
    n = Node(NET_IP, 4000)

    # updateObjectApiIP(ALL_RESULT_FOLDERS_DIR,OBJECT_INFO_FILE_NAME)
    while (True):
        # n = Node("localhost", 3000)
        current_tasks_status = get_tasks_status(prev_tasks_status)
        prev_tasks_status = current_tasks_status
        completed_uuid = [task['uuid'] for task in current_tasks_status if task['just_completed'] == True]
        if (len(completed_uuid) > 0):
        # if (True):
            print("Task with UUID {} was just completed!".format(completed_uuid[0]))
            task = n.get_task(completed_uuid[0])
            resuls_folder_path = download_assets(task)
            objInfo = get_object_info(resuls_folder_path)
            objUrl = objInfo['object_url']
            objUpperLeftCoord = objInfo['upper_left_coord']
            print("Object path: {}".format(objUrl))
            print("objUpperLeftCoord: {}".format(objUpperLeftCoord))
            print("TASK INGO: ",task.info())


            output_data = {
                    'object_url': objUrl,
                'upper_left_coord': objUpperLeftCoord
            }
            algorithmObj = {
                "algorithm_name":models.Algorithm.CREATE_3D_OBJECT_ALGORITHM,
                "output": output_data,
                "input":{'test_data':'test_value'},
                'user': get_user_model().objects.all().first().id,
                'operation':models.Operation.objects.all().first().id,
                'canBeLoadedOnMap':True
            }
            # newAlgorithm = models.Algorithm(**algorithmObj)
            views.AlgorithmRetrieveView.save_algorithm_to_db(algorithmObj)

            # objRoute = os.path.relpath(objLocalPath,PROJECT_DIR)
            # obj_uri = os.path.join(WEBSERVER_URL, objRoute)
            # obj_info = {'uuid':completed_uuid[0],'uri': obj_uri, 'coords': objUpperLeftCoord, 'just_completed':True}
            #
            # json_file_path = os.path.join(results_folder_path, OBJECT_INFO_FILE_NAME)
            # payload = json.dumps(obj_info, indent=4)
            # with open(json_file_path, 'w') as outfile:
            #     outfile.write(payload)
            # url = API_URL + "/odm/results"
            # requests.request("POST", url, headers={'Content-Type': 'application/json'}, data=payload)
        # for uuid in task_uuids:
        time.sleep(10)
def main():

    global NODE_ODM_API_URL, NET_IP

    if (len(sys.argv) < 2):
        print("\nPlease make sure you provide the following arguments:"
              "\n1)NODE_ODM_API_URL "

              )
        exit()
    else:
        # API_URL = sys.argv[1]
        NODE_ODM_API_URL = sys.argv[1]
        NET_IP = sys.argv[2]
        # ALL_RESULT_FOLDERS_DIR = sys.argv[4]
        # PROJECT_DIR = sys.argv[5]
        # WEBSERVER_URL = sys.argv[6]
    logger.info('Waiting for a 3D object to be created.')
    start()
if __name__ == '__main__':
    main()


