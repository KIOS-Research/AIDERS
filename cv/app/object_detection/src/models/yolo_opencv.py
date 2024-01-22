import cv2
import os
import numpy as np
import logging
import numba as nb
import abc
from object_detection.src.utils.rect import to_tlbr, as_tlbr
import asyncio
"""
Darknet yolo detector class using OPENCV DNN library
"""

DET_DTYPE = np.dtype(
    [('tlbr', float, 4),
     ('label', int),
     ('conf', float)],
    align=True
)
# DET_DTYPE = np.dtype(
#     [('index', int),
#      ('tlwh', float, 4),
#      ('tlbr', float, 4),
#      ('label', int),
#      ('classN', str),
#      ('conf', float)],
#     align=True
# )
LOGGER = logging.getLogger(__name__)

class Detector(abc.ABC):
    @abc.abstractmethod
    def __init__(self, size):
        self.size = size


    @abc.abstractmethod
    def detect(self,frame):
        """
        Synchronizes, applies postprocessing, and returns a record array
        of detections (DET_DTYPE).
        This function should be called after `detect_async`.
        """
        raise NotImplementedError


class Yolo_detector_ocv(Detector):

    def __init__(self,size,weights_file=None,config_file=None,classes_file=None,conf_thresh=None,
                 nms_thresh=None,use_gpu=True,netsize=[512,512]):
        super().__init__(size)
        print("WEIGHTS FILE: ", weights_file)
        print("WEIGHTS FILE ABS PATH: ", os.path.abspath(weights_file))
        self.weights = weights_file
        self.config = config_file
        self.classes_file = classes_file
        self.gpu = use_gpu
        self.net  = cv2.dnn.readNet( self.weights, self.config )
        # self.net  = cv2.dnn.readNetFromDarknet(self.config, self.weights)
        if self.gpu:
            #set CUDA as the preferable backend and target
            LOGGER.info("[INFO] setting preferable backend and target to CUDA...")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL_FP16)

        ln = self.net.getLayerNames()
        self.layers = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.netsize = netsize
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.detections=[]
        classes = []
        with open(self.classes_file, 'r') as f:
            classes = [line.strip() for line in f.readlines()]
        self.labels = np.array(classes,dtype=str)
        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(size=(self.netsize[0], self.netsize[1]), scale=1 / 256)


    def detect(self, frame):
        self.frame = frame
        image = self._preprocess(frame)
        try:
            classes, scores, boxes = self.model.detect(image, self.conf_thresh, self.nms_thresh)
        except Exception as e:
            LOGGER.error("[ERROR ]"+e)
        if len(boxes)==0:
            detections = []
            detections = np.asarray(detections, dtype=DET_DTYPE).view(np.recarray)
            return detections
            # return [],[],[],[]

        fixboxes = self.de_normalize(boxes,self.netsize,self.size)
        boxes = np.array(fixboxes,dtype=np.int32)

        detections = self.postprocess( classes[:,0], scores[:,0], boxes,self.labels)
        detections = np.asarray(detections, dtype=DET_DTYPE).view(np.recarray)
        self.detections = detections
        return detections

    def get_dets(self): #testing
        return self.detections

    def _preprocess(self,frame):
        self.imgsize = [frame.shape[1],frame.shape[0]]
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return cv2.resize(img, (self.netsize[0], self.netsize[1]), interpolation=cv2.INTER_LINEAR)

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def postprocess(classes, scores, boxes,labels):
        dets = []
        for i in range(0,len(boxes)):
            label = labels[classes[i]]
            tlbr = as_tlbr(to_tlbr(boxes[i]))
            dets.append((tlbr, classes[i], scores[i]))
        return dets

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def de_normalize(boxes,netsize,imgsize):
        finBoxes = []
        for i in range(len(boxes)):
            box = boxes[i]
            x,y,w,h= int((box[0] / netsize[0]) * imgsize[0]),int((box[1] / netsize[1]) * imgsize[1]),\
                     int((box[2] / netsize[0]) * imgsize[0]), int((box[3] / netsize[1]) * imgsize[1])
            finBoxes.append((x,y,w,h))
        return finBoxes