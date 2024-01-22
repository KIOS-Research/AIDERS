# import pdb; pdb.set_trace()
from logic.algorithms.live_stream import videoio
import os
import sys
from logic.Constants import Constants
import time
from aiders import models, serializers, views
from django.shortcuts import get_object_or_404
from django.core.files.base import ContentFile
import cv2
droneStreamingObj = {}


def capture_feed(drone_name):
    global droneStreamingObj
    resize_to = Constants.LIVE_STREAM_CONFIG.get('resize_to')
    rtmp_uri = Constants.LIVE_STREAM_CONFIG.get(
        'hls_uri').replace('drone_id', drone_name)
    resolution = Constants.LIVE_STREAM_CONFIG.get('resolution')
    frame_rate = Constants.LIVE_STREAM_CONFIG.get('frame_rate')
    buffer_size = Constants.LIVE_STREAM_CONFIG.get('buffer_size')
    print("RTMP URI: ", rtmp_uri)

    retryTimes = 3
    for i in range(retryTimes, 0, -1):
        try:
            droneStreamingObj[drone_name] = videoio.VideoIO(size=resize_to, input_uri=rtmp_uri, output_uri=None,
                                                            resolution=resolution,
                                                            frame_rate=frame_rate, buffer_size=buffer_size)
            droneStreamingObj[drone_name].start_capture()
            break
        except RuntimeError as e:
            if (i == 1):
                print("Could not start video stream after {} retries.".format(
                    str(retryTimes)))
                exit(-1)
            print("INSTSANCES: videoio.VideoIO.instances1",
                  videoio.VideoIO.instances)
            videoio.VideoIO.instances = []
            print("INSTSANCES: videoio.VideoIO.instances2",
                  videoio.VideoIO.instances)

            print(
                "Could not start video stream. Retrying {} more times...".format(str(i - 1)))

    print("Started capturing the video live stream for drone {}".format(drone_name))
    qs = models.Drone.objects.filter(drone_name=drone_name)
    drone = get_object_or_404(qs)
    live_stream_session = views.LiveStreamAPIOperations.create_live_stream_session_on_db(
        drone=drone)
    imgIdCounter = 1
    while True:
        currentFrame = droneStreamingObj[drone_name].read()
        success, frame_jpg = cv2.imencode('.jpg', currentFrame)
        content = frame_jpg.tobytes()
        frame_name = "frame{}.jpg".format(imgIdCounter)
        frame_file = ContentFile(content, name=frame_name)
        frameObj = views.LiveStreamAPIOperations.save_raw_frame_to_db(
            frame_file, drone_name, live_stream_session)
        views.LiveStreamAPIOperations.update_latest_raw_frame(
            live_stream_session, frameObj.frame.url)
        imgIdCounter += 1
        time.sleep(0.2)


def main(drone_name):
    print("Live_stream script will now start for drone {}!".format(drone_name))
    capture_feed(drone_name)
