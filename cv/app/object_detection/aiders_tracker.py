#!/usr/bin/env python3
import json
import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2

# from libs import videoio
import database.queries
import pytz
from object_detection.src import mot
from object_detection.src.utils import decoder, file_handler, profiler, rect

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

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
                 droneId=None,
                 operationId=None,
                 user = None,
                 verbose=0,
                 detection_type_str=None,
                 session=None,
                 drone_name = None
                 ):

        # draw  = 1 or output_file is not None
        draw=True
        self.droneId = droneId
        self.operationId = operationId
        self.gpuAvailable = True if os.environ.get("NVIDIA_AVAILABLE", "0") == "1" else False
        self.user = user
        self.mot_stop = 0
        self.output_file = output_file
        self.detection_session = None
        self.detection_type_str = detection_type_str
        self.session = session
        self.drone_name = drone_name

        # initialize config file
        default_cfg = Path(__file__).parent / 'src' / 'models' / 'cfg' / 'config.json'
        with open(default_cfg) as cfg_file:
            self.config = json.load(cfg_file, cls=decoder.ConfigDecoder, object_hook=lambda d: SimpleNamespace(**d))

        self.detection_type = self.get_detector_model(self.detection_type_str)
        
        # initialize stream class
        # self.stream = videoio.VideoIO(self.config.resize_to, input_file, output_file, **vars(self.config.stream_cfg))

        # initialize file handler class
        self.export = file_handler.File_handler(**vars(self.config.export_cfg))

        # set up logging
        logging.basicConfig(format='%(asctime)s [%(levelname)8s] %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')
        log_file = f'results_TM/verbose_log' \
                   f'{datetime.now(timezone).strftime("%Y-%m-%dT%H:%M")}.log'
        self.logger = logging.getLogger(mot.__name__)
        self.log_handler = logging.FileHandler(log_file)
        if verbose == 1:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)
        self.logger.addHandler(self.log_handler)

        print(self.detection_type)

        # initialize multi object tracker class
        self.mot = mot.MOT(self.config.resize_to, self.export, 30, self.detection_type,
                           **vars(self.config.mot_cfg), draw=draw)
        self.mot.reset(1/5)

        # start capturing the rtmp stream through videoio
        # self.stream.start_capture()
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


    def _run_tracker(self):
        frames_folder_name = f"detection_session{self.session}_{self.drone_name}"
        frames_path = os.path.join("/media", frames_folder_name)
        try:
            os.makedirs(frames_path)
        except:
            pass
        dji_drone = None

        try:
            with profiler.Profiler('app') as prof:
                with self.cond:
                    startTime = time.time()
                    detectionInterval = 1 / int(os.environ.get("COMPUTER_VISION_FPS"))  # run detection X frames per second
                    nextDetectionTime = startTime + detectionInterval                    
                    frameCounter = 0
                    
                    while not self.exit_event.is_set() or cv2.getWindowProperty(self.video_window , 0) >= 0:
                        currentTime = time.time()
                        if currentTime >= nextDetectionTime:
                            frameCounter+=1
                            nextDetectionTime += detectionInterval

                            latestLiveStreamFrame = database.queries.getLatestLiveStreamFrame(self.droneId)
                            currentFrame = cv2.imread(latestLiveStreamFrame[0])
                            # currentFrame = self.stream.read()

                            if currentFrame is None:
                                break

                            if self.mot_stop == 1:
                                break
                            else:
                                self.mot.step(currentFrame,dji_drone)

                                # prepare frame filename
                                currentDateTime = datetime.now(timezone).time()
                                formattedDateTime = currentDateTime.strftime('%H-%M-%S')
                                frame_name = f"det-frame{frameCounter:05d}_{formattedDateTime}.jpg"

                                # if self.mot.frame_has_detected_object: # save only frames with detected objects
                                framePath = f"{frames_path}/{frame_name}"

                                try:
                                    cv2.imwrite(framePath, currentFrame)
                                    # cv2.imwrite(framePath, cv2.resize(currentFrame, (720, 405)))
                                    frameId = database.queries.saveFrame(self.session, framePath) # save the frame info in the database
                                    # print(f"Image saved: {framePath}")
                                except Exception as e:
                                    print(f"Error saving image: {e}")

                                # THE FRAME AT THIS POINT HAS THE BOUNDING BOXES

                                imH, imW = currentFrame.shape[:2]

                                # run only if frame has detected objects
                                if self.mot.frame_has_detected_object:
                                    telemetry = database.queries.getDroneLatestTelemetry(self.droneId) # get latest drone telemetry
                                    # exporting results
                                    for track in self.mot.visible_tracks():

                                        # pernw self.tlbr kai to stelnw sto to_tlwh
                                        #x, y = get_center(self.tlbr)
                                        x,y,w,h = rect.get_center_wh(track.tlbr)

                                        # calculate detected object position
                                        # telemetry - lat, lon, alt, heading, gimbal_angle
                                        dist_gps, lat2, long2 = track.calc_dist_gps_coords(
                                            (x, y, w, h), telemetry[2], (telemetry[0], telemetry[1]), telemetry[3], telemetry[4], imH, imW
                                        )

                                        # save the detected object
                                        database.queries.saveDetectedObject(lat2, long2, track.lbl_str, track.trk_id, dist_gps, self.operationId, self.droneId, self.session, frameId)

                                        # check if it is the first detection
                                        # csv export for new tracks
                                        if track.is_exported == 0:
                                            self.export.export_vehicle(track, self.config.resize_to)
                                            self.export.export_track(track, self.config.resize_to)
                                            track.file_exported()
                                        else:
                                            self.export.export_track(track, self.config.resize_to)

                    self.cond.notify()

        finally:
            # clean up resources
            # self.stream.release()
            avg_fps = round(self.mot.frame_count / prof.duration)
            self.logger.info('Average FPS: %d', avg_fps)
            self.mot.print_timing_info(self.mot.frame_count)


    def get_detector_model(self, det_type_str):
        if (det_type_str == "VEHICLE_DETECTOR"):
            self.config.mot_cfg.class_ids = [0, 1, 2, 3]
            if self.gpuAvailable:
                # return "YOLOOCV" # TODO: TEMP TO TEST ON CPU
                return "YOLO"
            else:
                return "YOLOOCV"
        elif (det_type_str == "VEHICLE_PERSON_DETECTOR"):
            self.config.mot_cfg.class_ids = [0, 1, 2, 3, 4]
            if self.gpuAvailable:
                return "YOLO_BATCH"
            else:
                pass
                #TODO: RETURN AN ERROR IF USER WANTS TO START THIS DETECTOR
                #  WITHOUT GPU CAPABILITIES
        elif (det_type_str == "NO_ACTIVE_DETECTOR"):
            return "NO_ACTIVE_MODEL"
        
        
    # class method to access the get method without any instance
    @classmethod
    def get(cls, drone_name):
        for inst in cls.instances:
            if inst.drone_name == drone_name:
                    return inst
        return None
