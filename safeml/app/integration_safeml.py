import os
from ultralytics import YOLO
import cv2
import time
from datetime import datetime
import pytz



import database.queries
from safeml.monitor import SafeMLMonitor

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

class SafeMLDetection():
    def __init__(self, droneId, operationId, userId, sessionId, droneName):
        self.droneId = droneId
        self.operationId = operationId
        self.userId = userId
        self.sessionId = sessionId
        self.droneName = droneName

        self.running = True
    
    def start(self):
        # Integrate the model SafeML
        trained_weights = "/app/safeml/weights-medium/best.pt"
        trained_feature_paths = "/app/safeml/29092023_Good_Results/wasserstein/train_samples_instance.npy"
        distances_path = "/app/safeml/29092023_Good_Results/wasserstein/wasserstein_dist.npy"
        accuracy_path = "/app/safeml/29092023_Good_Results/wasserstein/wasserstein_acc.npy"
        pca_train = "/app/safeml/29092023_Good_Results/wasserstein/train_features.npy"
        model = YOLO(trained_weights, task="detect")
        monitor = SafeMLMonitor(trained_weights, trained_feature_paths, distances_path, accuracy_path, pca_train)

        output_folder = os.path.join("/media", f"safeml_session{self.sessionId}_{self.droneName}")
        os.makedirs(output_folder, exist_ok=True)

        startTime = time.time()
        detectionInterval = 1 / int(os.environ.get("COMPUTER_VISION_FPS")) # run detection X frames per second
        nextDetectionTime = startTime + detectionInterval       
        frameCounter = 0
        
        while self.running:
            currentTime = time.time()
            if currentTime >= nextDetectionTime:
                frameCounter+=1
                nextDetectionTime += detectionInterval        

                latestLiveStreamFrame = database.queries.getLatestLiveStreamFrame(self.droneId)
                image_path = latestLiveStreamFrame[0]

                # prepare frame filename
                currentDateTime = datetime.now(timezone).time()
                formattedDateTime = currentDateTime.strftime('%H-%M-%S')
                frame_name = f"safeml-frame{frameCounter:05d}_{formattedDateTime}.jpg"
                output_image_path = os.path.join(output_folder, frame_name)

                # Read the image in CV2 format
                frame = cv2.imread(image_path)
                res = model(frame, verbose=False)
                scue = monitor.update(frame)
                if scue is not None:
                    display_string = " SCUE : " +  "{:.3f}".format(scue)
                else:
                    display_string = " SCUE : Need more images " + str(monitor.count_inputs)

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.0  # Increase the text font size
                font_color = (255, 255, 255)  # Text color in BGR format
                font_thickness = 2  # Increase the text thickness
                x, y = 10, frame.shape[0] - 10  # Bottom left corner

                # Get the size of the text box to calculate background position
                text_size, _ = cv2.getTextSize(display_string, font, font_scale, font_thickness)
                text_width, text_height = text_size

                # Calculate the background position
                background_x_start = 0
                background_y_start = y - text_height -10
                background_x_end = x + text_width + 10
                background_y_end = y + 10
                # Create a transparent gray background only under the text
                overlay = frame.copy()
                alpha = 0.5  # Set the transparency level (0.0 for fully transparent, 1.0 for fully opaque)
                background_color = (128, 128, 128)  # Gray color in BGR format
                cv2.rectangle(overlay, (background_x_start, background_y_start), ( background_x_end, background_y_end), background_color, -1)  # -1 for filled rectangle

                # Add the overlay to the image with the specified alpha
                cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
                cv2.putText(frame, display_string, (x, y), font, font_scale, font_color, font_thickness)
                cv2.imwrite(output_image_path, frame)

                frameId = database.queries.saveFrame(self.sessionId, output_image_path)
                database.queries.saveSafeMLOutput(self.sessionId, frameId, scue)
                
        print(f"{self.droneName}: SafeMl detection stopped.")

    def stop(self):
        self.running = False