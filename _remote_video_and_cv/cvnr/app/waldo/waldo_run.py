import math
import os
import random
import re
import time
import uuid
from datetime import datetime

import cv2
import database.queries
import numpy as np
import onnxruntime as rt
import pytz
import torch

timezone = pytz.utc


def generate_uuid():
    return str(uuid.uuid4())


frame_count = 0

uuid_of_vid_base = generate_uuid()


class ObjectDetector:
    def __init__(self, droneId, operationId, userId, sessionId, droneName) -> None:
        self.droneId = droneId
        self.operationId = operationId
        self.userId = userId
        self.sessionId = sessionId
        self.droneName = droneName
        self.stop = False
        self.model = os.path.join(
            "/app/waldo", "yolov7-W25_rect_1280_736_newDefaults-bs96-best-topk-200.onnx"
        )
        self.tempcount = 0
        self.sess, self.input_name = self.init_session()

    def resize_and_pad(self, frame, expected_width, expected_height):
        if expected_width == expected_height:
            height, width, _ = frame.shape
            new_dim = min(height, width)
            start_x = (width - new_dim) // 2
            start_y = (height - new_dim) // 2
            frame = frame[start_y : start_y + new_dim, start_x : start_x + new_dim]

        ratio = min(expected_width / frame.shape[1], expected_height / frame.shape[0])
        new_width = int(frame.shape[1] * ratio)
        new_height = int(frame.shape[0] * ratio)
        frame = cv2.resize(frame, (new_width, new_height))
        padded_frame = np.zeros((expected_height, expected_width, 3), dtype=np.uint8)
        y_offset = (expected_height - new_height) // 2
        x_offset = (expected_width - new_width) // 2
        padded_frame[
            y_offset : y_offset + new_height, x_offset : x_offset + new_width
        ] = frame

        return padded_frame

    def init_session(self):
        cuda = torch.cuda.is_available()

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if cuda
            else ["CPUExecutionProvider"]
        )
        sess = rt.InferenceSession(self.model, providers=providers)
        input_name = sess.get_inputs()[0].name

        return sess, input_name

    def pixel_to_gps(self, pixel, img_size, fov, drone_info):
        # Unpack drone_info
        lat, lon, alt, bearing, pitch = drone_info

        # Calculate the angle of the pixel from the center of the image
        dx = (pixel[0] - img_size[0] / 2) / (img_size[0] / 2) * (fov[0] / 2)
        dy = (
            ((img_size[1] - pixel[1]) - img_size[1] / 2)
            / (img_size[1] / 2)
            * (fov[1] / 2)
        )

        # Adjust the angles for the pitch of the camera
        dy += pitch

        # Calculate the relative position of the pixel from the drone
        dx = alt * math.tan(math.radians(dx))
        dy = alt * math.tan(math.radians(dy))

        # Rotate the relative position by the drone's bearing
        dx, dy = dx * math.cos(math.radians(-bearing)) - dy * math.sin(
            math.radians(-bearing)
        ), dx * math.sin(math.radians(-bearing)) + dy * math.cos(math.radians(-bearing))

        # Convert the relative position to GPS coordinates
        lat += dy / 111111
        lon += dx / (111111 * math.cos(math.radians(lat)))

        return lat, lon

    def get_resolution_from_model_path(self, model_path):
        # Check for rectangular pattern first
        rect_match = re.search(r"_rect_(\d+)_(\d+)_", model_path)
        if rect_match:
            return int(rect_match.group(1)), int(rect_match.group(2))

        # Check for the square pattern next
        square_match = re.search(r"(\d+)px", model_path)
        if square_match:
            res = int(square_match.group(1))
            return res, res  # Return height and width

        return None, None  # If neither match, return None for both dimensions

    def get_max_outputs(self):
        max_outputs_match = re.search(r"topk.\d+", self.model)
        max_outputs = None
        if max_outputs_match:
            max_outputs = int(re.search(r"\d+", max_outputs_match.group(0)).group(0))

        return max_outputs

    def _scale_based_on_bbox(self, bbox):
        # Compute the diagonal length of the bounding box
        diag_length = np.sqrt((bbox[2] - bbox[0]) ** 2 + (bbox[3] - bbox[1]) ** 2)

        # Linearly scale the text size and thickness based on the diagonal length
        text_size = max(0.4, diag_length / 300)

        return text_size

    def process_frame(self, frame, sess, max_outputs):
        colors = {
            "car": (255, 0, 0),  # Blue
            "van": (255, 0, 0),  # Cyan
            "truck": (0, 255, 0),  # Green
            "building": (0, 42, 92),  # Brown
            "human": (203, 192, 255),  # Pink
            "gastank": (0, 255, 255),  # Yellow
            "digger": (0, 0, 255),  # Red
            "container": (255, 255, 255),  # White
            "bus": (128, 0, 128),  # Purple
            "u_pole": (255, 0, 255),  # Magenta
            "boat": (0, 0, 139),  # Dark red
            "bike": (144, 238, 144),  # Light green
            "smoke": (0, 230, 128),  # Grey
            "solarpanels": (0, 0, 0),  # Black
            "arm": (0, 0, 0),  # Black
            "plane": (255, 255, 255),  # White
        }
        names = [
            "car",
            "van",
            "truck",
            "building",
            "human",
            "gastank",
            "digger",
            "container",
            "bus",
            "u_pole",
            "boat",
            "bike",
            "smoke",
            "solarpanels",
            "arm",
            "plane",
        ]

        resolution = 960

        image = frame.copy()
        image = image.transpose((2, 0, 1))
        image = np.expand_dims(image, 0)
        image = np.ascontiguousarray(image)

        im = image.astype(np.float32)
        im /= 255

        inp = {self.input_name: im}
        outputs = sess.run(None, inp)[0]
        thickness = 1
        category_counts = {}
        box_center_points = []

        for i, (batch_id, x0, y0, x1, y1, cls_id, score) in enumerate(outputs):
            center_x = (x0 + x1) / 2
            center_y = (y0 + y1) / 2
            box_center_points.append([center_x, center_y])

            box = np.array([x0, y0, x1, y1])
            box = box.round().astype(np.int32).tolist()
            cls_id = int(cls_id)
            score = round(float(score), 1)
            name = names[cls_id]
            color = colors[name]
            name += " " + str(score)

            text_size = self.scale_based_on_bbox(box)

            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, thickness)
            cv2.putText(
                frame,
                name,
                (box[0], box[1] - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                text_size,
                color,
                thickness=thickness,
            )
            if max_outputs is not None:
                cv2.putText(
                    frame,
                    f"ONNX network max Outputs: {max_outputs}",
                    (frame.shape[1] - 250, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 0, 255),
                    1,
                )

            # Get the name of the class without the score
            class_name = name.split()[0]

            # Increment the count for this class in the dictionary
            if class_name in category_counts:
                category_counts[class_name] += 1
            else:
                category_counts[class_name] = 1

        # Write the category counts on the frame
        y_position = 20  # Initial y position
        for category, count in category_counts.items():
            cv2.putText(
                frame,
                f"{category}: {count}",
                (10, y_position),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
            y_position += 20  # Increment the y position for the next text

        # Return the frame with detection boxes
        return frame, box_center_points

    def scale_based_on_bbox(self, bbox):
        # Compute the diagonal length of the bounding box
        diag_length = np.sqrt((bbox[2] - bbox[0]) ** 2 + (bbox[3] - bbox[1]) ** 2)

        # Linearly scale the text size and thickness based on the diagonal length
        text_size = max(0.4, diag_length / 300)

        return text_size

    def run_inference(self, img_path, output_img_path):
        input_frame_raw = cv2.imread(img_path)
        height, width, channels = input_frame_raw.shape
        expected_width, expected_height = self.get_resolution_from_model_path(
            self.model
        )  # Get expected resolution

        max_outputs = self.get_max_outputs()
        frame = self.resize_and_pad(input_frame_raw, expected_width, expected_height)
        out_frame, box_center_points = self.process_frame(
            frame=frame, sess=self.sess, max_outputs=max_outputs
        )

        cv2.imwrite(output_img_path, out_frame)
        frameId = database.queries.saveFrame(self.sessionId, output_img_path)

        # loop through all the detected points and calculate their coordinates
        telemetry = database.queries.getDroneLatestTelemetry(
            self.droneId
        )  # get latest drone telemetry

        # TODO: get fov from database and adjust for zoom level(?)
        fov_horizontal = 68  # FOR MAVIC
        fov_vertical = 40  # mavic

        gimbal_angle = telemetry[4] + 90

        detectedCoords = []
        for p in box_center_points:
            # telemetry - lat, lon, alt, heading, gimbal_angle
            lat, lon = self.pixel_to_gps(
                p,
                (width, height),
                (fov_horizontal, fov_vertical),
                (telemetry[0], telemetry[1], telemetry[2], telemetry[3], gimbal_angle),
            )
            detectedCoords.append([lat, lon])
            # TODO: save the output of Waldo to the database (new query - multiple rows)
            database.queries.saveDetectedObject(lat, lon, 'car', random.randint(1,10), 1.2, self.operationId, self.droneId, self.sessionId, frameId)

        # save the detected coordinates
        # if len(detectedCoords) > 0:
        #     database.queries.saveCrowdLocalizationResults(
        #         self.droneId, self.operationId, self.sessionId, frameId, detectedCoords
        #     )
        #     # print(detectedCoords, flush=True)

    def start_loop(self):
        total_count = []
        output_folder = os.path.join(
            "/media", f"detector_session{self.sessionId}_{self.droneName}"
        )
        os.makedirs(output_folder, exist_ok=True)
        startTime = time.time()
        detectionInterval = 1 / int(
            os.environ.get("COMPUTER_VISION_FPS")
        )  # run detection X frames per second
        nextDetectionTime = startTime + detectionInterval
        frameCounter = 0

        while not self.stop:
            currentTime = time.time()
            if currentTime >= nextDetectionTime:
                frameCounter += 1
                nextDetectionTime += detectionInterval

                latestLiveStreamFrame = database.queries.getLatestLiveStreamFrame(
                    self.droneId
                )
                image_path = latestLiveStreamFrame[0]

                # prepare frame filename
                currentDateTime = datetime.now(timezone).time()
                formattedDateTime = currentDateTime.strftime("%H-%M-%S")
                frame_name = f"detector-frame{frameCounter:05d}_{formattedDateTime}.jpg"
                self.run_inference(
                    img_path=image_path,
                    output_img_path=os.path.join(output_folder, frame_name),
                )

        print(f"{self.droneName}: Waldo detector stopped.")

    def stopDetector(self):
        self.stop = True
