from __future__ import division

import warnings

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional
from crowd_loc.Networks.HR_Net.seg_hrnet import get_seg_model
from torchvision import transforms

warnings.filterwarnings("ignore")
import os
import time
from collections import OrderedDict
import database.queries
from datetime import datetime
import pytz
timezone = pytz.utc


import math

class Crowd_local:
    def __init__(
        self, droneId, operationId, userId, sessionId, droneName
    ) -> None:
        self.droneId = droneId
        self.operationId = operationId
        self.userId = userId
        self.sessionId = sessionId
        self.droneName = droneName
        self.stop = False
        self.model = self.get_model()
        self.tempcount = 0

        self.weights_path = os.path.join("/app/crowd_loc/weights", "model_best_u-hrnet05.pth")

        if os.path.isfile(self.weights_path):
            self.load_weights(model=self.model)

    def get_model(self):
        model = get_seg_model()
        for param in model.parameters(): # Freeze model weights
            param.requires_grad = False
            
        model = model.cuda() # Load model onto GPU
        return model

    def load_weights(self, model):
        """A simple function to load the pre-trained weights onto the model.

        Args:
            model (): A pytorch module containing the layers and branches of the model
        """
        print("LOADING MODEL WEIGHTS")
        checkpoint = torch.load(self.weights_path)
        state_dict = checkpoint["state_dict"]
        ### CLEAN THIS LATER ###
        # TODO: Find a weight to edit the weights file permenately so this code block is not needed.
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            k = k[7:]
            new_state_dict[k] = v
        checkpoint["state_dict"] = new_state_dict
        ##############################
        model.load_state_dict(checkpoint["state_dict"])
        print("CHECKPOINT LOADED")

    def transform_input(self, frame):
        """Transform the input frame by normalizing it's values and turning it into a tensor.

        Args:
            frame (cv2 object): The frame that will go through the model for inference. It can be a cv2 object or a PIL Image.

        Returns:
            tensor: The image to be processed by the model
        """
        tensor_transform = transforms.ToTensor()
        frame = frame.copy()
        image = tensor_transform(frame).unsqueeze(0)

        return image

    def plot_points(self, points, img):
        """A simple function that takes the predicted points from the LMDS algorithm and draws them on the original frame.

        Args:
            points (numpy array): A numpy array with the predicted points. Each element of the array should have 0 or 1 depending if a point is there or not
            img (cv2 object): The original image to draw the points on

        Returns:
            cv2 object: Returns the original frame back with the points on it for each prediction
        """
        out_points_array = []
        point_cord = np.nonzero(points)

        if len(point_cord[0]) != 0:
            for i in range(0, len(point_cord[0])):
                h = int(point_cord[0][i])
                w = int(point_cord[1][i])
                out_points_array.append([w, h])
                point_img = cv2.circle(img, (w, h), 3, (0, 255, 0), -1)
        else:
            point_img = img
        return point_img, out_points_array



    def LMDS_counting(self, input_img):
        """LMDS Counting and Localization algorithm.

        Args:
            input_img (tensor): This is the model's output

        Returns:
            int: The predicted counting number
            np.array: An array with the same size of the original input. Each element of the array corresponds to a pixel and its either 0 or 1 depending on the detection.
        """
        sigmoid_layer = nn.Sigmoid()
        input_img = sigmoid_layer(input_img)
        
        input_max = torch.max(input_img).item()

        if input_max < 0.1:
            input_img = input_img * 0

        local_max = nn.functional.max_pool2d(input_img, (3, 3), stride=1, padding=1)
        local_max = (local_max == input_img).float()
        input_img = local_max * input_img

        # Set threshold value based on the max value of input_img
        threshold = 100.0 / 255.0 * input_max
        input_img[
            input_img < threshold
        ] = 0  # Everything below this threshold, set to 0
        input_img[input_img > 0] = 1  # Everything above 0 set to 1

        count = int(torch.sum(input_img).item())
        kpoint = input_img.data.squeeze(0).squeeze(0).cpu().numpy()

        return count, kpoint




    def pixel_to_gps(self, pixel, img_size, fov, drone_info):
        # Unpack drone_info
        lat, lon, alt, bearing, pitch = drone_info

        # Calculate the angle of the pixel from the center of the image
        dx = (pixel[0] - img_size[0] / 2) / (img_size[0] / 2) * (fov[0] / 2)
        dy = ((img_size[1] - pixel[1]) - img_size[1] / 2) / (img_size[1] / 2) * (fov[1] / 2)

        # Adjust the angles for the pitch of the camera
        dy += pitch

        # Calculate the relative position of the pixel from the drone
        dx = alt * math.tan(math.radians(dx))
        dy = alt * math.tan(math.radians(dy))

        # Rotate the relative position by the drone's bearing
        dx, dy = dx * math.cos(math.radians(-bearing)) - dy * math.sin(math.radians(-bearing)), dx * math.sin(math.radians(-bearing)) + dy * math.cos(math.radians(-bearing))

        # Convert the relative position to GPS coordinates
        lat += dy / 111111
        lon += dx / (111111 * math.cos(math.radians(lat)))

        return lat, lon


    
    def run_inference(
        self,
        img_path,
        output_img_path,
        total_count: list,
    ):
        """Run inference on a given frame

        Args:
            img_path (_type_): Path for the frame to run inference on
            output_img_path (_type_): Path to save the output frame
            total_count (list): An empty list to append the predicted count in order to get the average
            prev_frame_time (_type_): Time since the last frame. Needed to calculate FPS
            new_frame_time (_type_): Time of the new frame. Needed to calculate FPS.
        """
        input_frame_raw = cv2.imread(img_path)
        height, width, channels = input_frame_raw.shape

        input_frame = self.transform_input(input_frame_raw)
        input_frame = input_frame.cuda()
        self.model.eval()

        with torch.no_grad():
            output_img = self.model(input_frame) # Pass input frame through model to get prediction
            
            pred_count, pred_points = self.LMDS_counting(input_img=output_img) # Run prediction through the LMDS algorithm
            out_frame, out_points_array = self.plot_points(points=pred_points, img=input_frame_raw)  # Draw points on the original frame


        total_count.append(pred_count)
        avg_count = int(np.mean(total_count))


        cv2.rectangle(input_frame_raw, (0, height), 
                    (310, height-100), 
                    (0, 0, 0), -1)

        cv2.putText(
            out_frame,
            "Count: " + str(pred_count),
            (30, height-60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            out_frame,
            "Avg Count: " + str(avg_count),
            (30, height-30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.imwrite(output_img_path, out_frame)
        frameId = database.queries.saveFrame(self.sessionId, output_img_path)

        # loop through all the detected points and calculate their coordinates
        telemetry = database.queries.getDroneLatestTelemetry(self.droneId) # get latest drone telemetry


        # TODO: get fov from database and adjust for zoom level(?)
        fov_horizontal = 68  # FOR MAVIC
        fov_vertical = 40  # mavic


        gimbal_angle = telemetry[4] + 90

        detectedCoords = []
        for p in out_points_array:
            # telemetry - lat, lon, alt, heading, gimbal_angle
            lat, lon = self.pixel_to_gps(p, (width, height), (fov_horizontal, fov_vertical), (telemetry[0], telemetry[1], telemetry[2], telemetry[3], gimbal_angle))
            detectedCoords.append([lat, lon])

        # save the detected coordinates
        if(len(detectedCoords) > 0):
            database.queries.saveCrowdLocalizationResults(self.droneId, self.operationId, self.sessionId, frameId, detectedCoords)
            # print(detectedCoords, flush=True)




    def start_loop(self):
        total_count = []
        output_folder = os.path.join(
            "/media", f"crowd_loc_session{self.sessionId}_{self.droneName}"
        )
        os.makedirs(output_folder, exist_ok=True)
        startTime = time.time()
        detectionInterval = 1 / int(os.environ.get("COMPUTER_VISION_FPS"))  # run detection X frames per second
        nextDetectionTime = startTime + detectionInterval
        frameCounter = 0

        while not self.stop:
            currentTime = time.time()
            if currentTime >= nextDetectionTime:
                frameCounter += 1
                nextDetectionTime += detectionInterval

                latestLiveStreamFrame = database.queries.getLatestLiveStreamFrame(self.droneId)
                image_path = latestLiveStreamFrame[0]

                # prepare frame filename
                currentDateTime = datetime.now(timezone).time()
                formattedDateTime = currentDateTime.strftime("%H-%M-%S")
                frame_name = (f"crowd_loc-frame{frameCounter:05d}_{formattedDateTime}.jpg")
                self.run_inference(
                    img_path=image_path,
                    output_img_path=os.path.join(output_folder, frame_name),
                    total_count=total_count,
                )

        print(f"{self.droneName}: Crowd Localization stopped.")



    def stopDetector(self):
        self.stop = True