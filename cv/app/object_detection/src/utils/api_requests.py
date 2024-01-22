import numpy as np
import json
import requests
import os
from .file_handler import  make_json
#     path('video_info', views.addVideoInfo),
#     path('vehicles/<int:video_id>', views.addVehicles),
#     path('tracks/<int:video_id>', views.addTracks),
#     path('traffic/<int:video_id>', views.addAverageTraffic),
#     path('traffic/<str:timestamp>/<int:video_id>', views.getAverageTraffic),
#
class API_Requests():
    def __init__(self, files={}, api_url='127.0.0.1/api'):
        # self.timestamp = timestamp
        for key in files:
            if os.path.normpath(files[key]).endswith('.csv'):
                json_path = files[key].replace('.csv','.json')
                make_json(files[key], json_path)
                files[key] = json_path
        self.files = files
        self.video_id = 0
        self.API_URL = api_url
        self.headers = {'content-type': 'application/x-www-form-urlencoded'}


    def post_videoinfo(self):
        f = open(self.files['video_info'])
        # returns JSON object as
        # a dictionary
        data = json.load(f)

        response = requests.post(self.API_URL+'/video_info/', json=data, headers=self.headers)
        # get the response in json format
        res_json = response.json()
        self.video_id = res_json['video_id']
        if res_json['message']== 'Success':
            return "Video added Successfully.\n"
        else:
            return "Failed adding Video Info."

    def post_all_data(self):
        f = open(self.files['video_info'])
        # returns JSON object as
        # a dictionary
        data = json.load(f)
        print(self.API_URL)
        print(data)
        try:
            response = requests.request(method='POST', url=self.API_URL+'/video_info', json=data)
            # get the response in json format
            res_json = response.json()
            print("video res",res_json)
            if res_json['message']== 'Success':
                self.video_id = res_json['video_id']
                msg = self.post_vehicles()
                return "Video added Successfully.\n" + msg
            else:
                return "Failed adding Video Info."
        except Exception as e:
            print('error in posting data : ',e)

    def post_vehicles(self):
        f = open(self.files['vehicles'] )
        # returns JSON object as
        # a dictionary
        data = json.load(f)
        response = requests.post(self.API_URL+f'/vehicles/{self.video_id}', json=data, headers=self.headers)
        # get the response in json format
        res_json = response.json()
        print("vehs res",res_json)

        if res_json['message'] == 'Success':
            msg = self.post_tracks()
            return "Vehicles added Successfully.\n" + msg
        else:
            return "Vehicles failed."

    def post_tracks(self):
        f = open(self.files['tracks'])
        # returns JSON object as
        # a dictionary
        data = json.load(f)
        response = requests.post(self.API_URL + f'/tracks/{self.video_id}', json=data, headers=self.headers)
        # get the response in json format
        res_json = response.json()
        print("tracks res",res_json)

        if res_json['message'] == 'Success':
            return "Tracks added Successfully."
        else:
            return "Tracks failed."