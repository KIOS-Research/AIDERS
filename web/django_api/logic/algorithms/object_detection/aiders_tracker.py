#!/usr/bin/env python3
import json
import logging
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2
from aiders import models, serializers, views
from django.conf import settings
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from django.shortcuts import get_object_or_404
from logic.algorithms.live_stream import videoio
# import src

from logic.algorithms.object_detection import src


# import src.models
from logic.algorithms.object_detection.src.utils import (ConfigDecoder,
                                                         File_handler,
                                                         Profiler, rect,
                                                         ros_callback)
from logic.Constants import Constants
from PIL import Image
from rest_framework import status
from rest_framework.response import Response

class DetectedObject:
    def __init__(self,lat,lon,distFromDrone, trk_id, label):
        self.lat = lat
        self.lon = lon
        self.distFromDrone = distFromDrone
        self.trk_id = trk_id
        self.label = label

class Tracker(object):
    instances = []
    def __init__(self, input_file,
                 output_file=None,
                 display=False,
                 drone=None,
                 operation=None,
                 user = None,
                 verbose=0,
                 detection_type_str=None
                 ):

        # draw  = 1 or output_file is not None
        draw=True
        self.drone = drone
        self.operation = operation
        self.gpuAvailable = True if os.environ.get("NVIDIA_AVAILABLE", "0") == "1" else False
        self.user = user
        self.video_window = f'Video_{self.drone.drone_name}'
        self.mot_stop = 0
        self.display = display
        self.output_file = output_file
        self.detection_session = None
        self.detection_type_str = detection_type_str



        # self.detection_type = getDetType Implement this. We have to get the detection type based on detection string
        # initialize config file
        default_cfg = Path(__file__).parent / 'src' / 'models' / 'cfg' / 'config.json'
        with open(default_cfg) as cfg_file:
            self.config = json.load(cfg_file, cls=ConfigDecoder, object_hook=lambda d: SimpleNamespace(**d))

        self.detection_type = self._get_detector_type(self.detection_type_str)
        # initialize stream class
        self.stream = videoio.VideoIO(self.config.resize_to, input_file, output_file, **vars(self.config.stream_cfg))

        # initialize file handler class
        self.export = File_handler(**vars(self.config.export_cfg))

        # set up logging
        logging.basicConfig(format='%(asctime)s [%(levelname)8s] %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')
        log_file = f'results_TM/verbose_log' \
                   f'{datetime.now().strftime("%Y-%m-%dT%H:%M")}.log'
        self.logger = logging.getLogger(src.__name__)
        self.log_handler = logging.FileHandler(log_file)
        if verbose == 1:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)
        self.logger.addHandler(self.log_handler)

        # initialize multi object tracker class
        self.mot = src.MOT(self.config.resize_to, self.export, self.stream.cap_fps, self.detection_type,
                           **vars(self.config.mot_cfg), draw=draw)
        self.mot.reset(self.stream.cap_dt)

        self.stream.start_capture()
        self.logger.info('Starting video capture...')

        self.cond = threading.Condition()
        self.exit_event = threading.Event()
        self.cap_thread = threading.Thread(target=self._run_tracker)


        Tracker.instances.append(self)
    def start_tracker(self):
        if not self.mot_stop:
            if not self.cap_thread.is_alive():
                self.cap_thread.start()

    def stop_tracker(self):
        """Stop capturing from file or device."""

        self.mot_stop = 1
        with self.cond:
            self.exit_event.set()
            self.cond.notify()
        self.cap_thread.join()
        views.DetectionAPIOperations.update_detection_status_on_db(self.drone, models.Detection.DetectionStatusChoices.DETECTION_DISCONNECTED, models.Detection.DetectionModelChoices.NO_ACTIVE_MODEL)
        views.DetectionAPIOperations.update_detection_session_end_time(self.detection_session)
    # def _update_detection_status_on_db(self, detection_status):
    #     qs = models.Detection.objects.filter(drone__drone_name = self.drone.drone_name).update(detection_status=detection_status)
    #     # detection = get_object_or_404(qs)
    #     # detection.detection_status = detection_status
    #     # serializer = serializers.DetectionSerializer(detection)
    #     return Response(status=status.HTTP_200_OK)


    # def _create_detection_session_on_db(self):
    #     self.detection_session = models.DetectionSession.objects.create(
    #         user=self.user,
    #         operation=self.operation,
    #         drone=self.drone
    #     )

    def _update_detection_session_end_time(self):
        end_time = datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ)
        models.DetectionSession.objects.filter(pk=self.detection_session.id).update(end_time=end_time, is_active=False)

    def _update_latest_frame(self, latest_frame_url):
        models.DetectionSession.objects.filter(pk=self.detection_session.id).update(latest_frame_url=latest_frame_url)

    # def _save_frame_to_db(self, frame_file):
    #     detFrame = models.DetectionFrame.objects.create(
    #         frame=frame_file,
    #         detection_session=self.detection_session,
    #     )
    #     return detFrame.frame.url
    def _run_tracker(self):
        frames_folder_name = Constants.DETECTION_FRAMES_DIR_NAME_PREFIX + self.drone.drone_name
        frames_path = os.path.join(default_storage.location, frames_folder_name)
        try:
            os.mkdir(frames_path)
        except:
            pass
        dji_drone = None

        if self.display == 1:
            cv2.namedWindow(self.video_window, cv2.WINDOW_AUTOSIZE)
        views.DetectionAPIOperations.update_detection_status_on_db(self.drone, models.Detection.DetectionStatusChoices.DETECTION_CONNECTED, self.detection_type_str)
        self.detection_session = views.DetectionAPIOperations.create_detection_session_on_db(self.user, self.operation, self.drone)
        # self.create_detection_session_on_db()
        try:
            with Profiler('app') as prof:
                with self.cond:
                    imgIdCounter = 0
                    while not self.exit_event.is_set() or cv2.getWindowProperty(self.video_window , 0) >= 0:
                        imgIdCounter+=1
                        currentFrame = self.stream.read()


                        if currentFrame is None:
                            break

                        if self.mot_stop == 1:
                            # self.mot.reset(self.stream.cap_dt)
                            break
                        else:

                            self.mot.step(currentFrame,dji_drone)

                            success, frame_jpg = cv2.imencode('.jpg', currentFrame)
                            content = frame_jpg.tobytes()
                            frame_name = "frame{}.jpg".format(imgIdCounter)
                            frame_file = ContentFile(content, name=frame_name)
                            frameObj = views.DetectionAPIOperations.save_frame_to_db(frame_file, self.detection_session)
                            # self.detectionAPIOperations.frame_file = ContentFile(content, name=frame_name)
                            # self.detectionAPIOperations.latest_frame_url =\
                            #     self.detectionAPIOperations.save_frame_to_db()
                            views.DetectionAPIOperations.update_latest_frame(self.detection_session, frameObj.frame.url)

                            # cv2.imwrite(frame_path, cv2.resize(frame, (720, 405)))


                            #THE FRAME AT THIS POOINT HAS THE BOUNDING BOXES


                            # visible = len(list(self.mot.visible_tracks()))
                            # mean_velocity = float("{:.2f}".format(mot.mean_velo()))
                            # labels_dict = mot.labels()
                            # label_names = get_labels_all()
                            # for label in label_names:
                            #     label_value = labels_dict.get(label, 0)
                            #     labels_dict[label] = label_value
                            # labels_keys = list(labels_dict.keys())
                            # labels_values = list(labels_dict.values())
                            imH, imW = currentFrame.shape[:2]
                            # exporting results
                            # TODO Export class be changed to posting the data to the API
                            for track in self.mot.visible_tracks():

                                #pernw self.tlbr kai to stelnw sto to_tlwh
                                #x, y = get_center(self.tlbr)
                                x,y,w,h = rect.get_center_wh(track.tlbr)
                                telemetry = models.Telemetry.objects.filter(drone__drone_name=self.drone.drone_name).last()


                                # droneTelemetry = models.Telemetry
                                dist_gps, lat2, long2 = track.calc_dist_gps_coords(
                                    (x, y, w, h), telemetry.alt, (telemetry.lat, telemetry.lon), telemetry.heading, telemetry.gimbal_angle, imH, imW
                                )

                                detObj = DetectedObject(lat2,long2, dist_gps, track.trk_id, track.lbl_str)
                                views.DetectionAPIOperations.save_detected_object_to_db(self.detection_session,detObj, frameObj)
                                # # check if it is the first detection
                                # # csv export for new tracks

                                if track.is_exported == 0:
                                    self.export.export_vehicle(track, self.config.resize_to)
                                    self.export.export_track(track, self.config.resize_to)
                                    # self.export.export_videoinfo(input_file, stream.cap_fps, config.resize_to,
                                    #                         mot.get_total_duration(),
                                    #                         mot.tracker.dji_drone.ros_telemetry.altitude,
                                    #                         mot.tracker.dji_drone.ros_telemetry.latitude,
                                    #                         mot.tracker.dji_drone.ros_telemetry.longitude, mot.tracker.ppkm)
                                    track.file_exported()
                                else:
                                    self.export.export_track(track, self.config.resize_to)

                        if self.display:
                            cv2.imshow(self.video_window , cv2.resize(frameObj, (1280,720)))
                            if cv2.waitKey(1) & 0xFF == 27 or cv2.getWindowProperty(self.video_window , cv2.WND_PROP_ASPECT_RATIO) < 0:
                                self.exit_event.set()
                                self.cond.notify()
                                break

                        if self.output_file is not None:
                            self.stream.write(frameObj)
                    self.cond.notify()

        finally:
            # TODO END UP CONNECTION WITH THE DRONE
            # clean up resources
            self.stream.release()
            # cv2.destroyWindow(self.video_window )
            avg_fps = round(self.mot.frame_count / prof.duration)
            self.logger.info('Average FPS: %d', avg_fps)
            self.mot.print_timing_info(self.mot.frame_count)


    def _get_detector_type(self, det_type_str):

        if (det_type_str == models.Detection.DetectionTypeStrChoices.VEHICLE_DETECTOR):
            self.config.mot_cfg.class_ids = [0, 1, 2, 3]
            if self.gpuAvailable:

                return models.Detection.DetectionModelChoices.YOLO
            else:
                return models.Detection.DetectionModelChoices.YOLOCV
        elif (det_type_str == models.Detection.DetectionTypeStrChoices.PERSON_AND_VEHICLE_DETECTOR):
            self.config.mot_cfg.class_ids = [0, 1, 2, 3, 4]
            if self.gpuAvailable:
                return models.Detection.DetectionModelChoices.YOLO_BATCH
            else:
                pass
                #TODO: RETURN AN ERROR IF USER WANTS TO START THIS DETECTOR
                #  WITHOUT GPU CAPABILITIES
        elif (det_type_str == models.Detection.DetectionTypeStrChoices.NO_ACTIVE_DETECTOR):
            return models.Detection.DetectionModelChoices.NO_ACTIVE_MODEL
    # class method to access the get method without any instance
    @classmethod
    def get(cls, drone_name):
        for inst in cls.instances:
            if inst.drone.drone_name == drone_name:
                    return inst
        return None
# tracker.start_tracker()
# tracker2.start_tracker()
# print('ekame')
# time.sleep(10)
# print('eprepe na stamatisi')
# tracker.stop_tracker()
# time.sleep(5)
# print('stop second one')
# tracker2.stop_tracker()
