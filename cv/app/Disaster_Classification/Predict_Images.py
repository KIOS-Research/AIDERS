# -*- coding: utf-8 -*-
"""
Created on Tue Jul 25 15:50:35 2023
@author: dshian01
"""

import os
import torch
from torchvision import transforms
from PIL import Image
from torchvision import models
from torchvision import transforms
from collections import OrderedDict
import os
import torch.nn as nn
import cv2
import random
import numpy as np
import time
from datetime import datetime
import pytz

import database.queries

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))


class DisasterClassification:

    def __init__(self, droneId, operationId, userId, sessionId, droneName):
        self.droneId = droneId
        self.perationId = operationId
        self.userId = userId
        self.sessionId = sessionId
        self.droneName = droneName

        self.stop = False
        # Call this function before creating your model or performing any training/inference
        self.set_random_seed(seed_value=42)


    def set_random_seed(self, seed_value=42):
        # Set Python random seed
        random.seed(seed_value)

        # Set NumPy random seed
        np.random.seed(seed_value)

        # Set PyTorch random seed
        torch.manual_seed(seed_value)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False





    def check_gpu_cuda(self):
        # Check if CUDA is available
        if torch.cuda.is_available():
            # Get the number of available GPUs
            num_gpus = torch.cuda.device_count()
            # Print the number of available GPUs
            print("Number of available GPUs:", num_gpus)
            # Iterate over each GPU and print its details
            for i in range(num_gpus):
                gpu_name = torch.cuda.get_device_name(i)
                print("GPU", i, ":", gpu_name)
            # Select GPU 0
            device = torch.device("cuda:0")
            print("Using CUDA GPU 0:", torch.cuda.get_device_name(device))
            print("CUDA Version:", torch.version.cuda)
        else:
            num_gpus = 0
            print("CUDA is not available.")
        return num_gpus



    def load_checkpoint(self, filepath, num_gpus):
        if num_gpus == 0:
            checkpoint = torch.load(filepath, map_location=torch.device('cpu'))
        else:
            checkpoint = torch.load(filepath)
        num_classes = 4
        print('Model Architecture:',checkpoint['arch'])
        if checkpoint['arch'] == 'efficientnet':
            model = models.efficientnet_b0(pretrained=False)
            for param in model.parameters():
                param.requires_grad = False
            model.class_to_idx = checkpoint['class_to_idx']
            in_features = model.classifier[-1].in_features
            classifier = nn.Sequential(OrderedDict([
                ('fc1', nn.Linear(in_features, 512)),
                ('relu', nn.ReLU()),
                ('drop', nn.Dropout(p=0.5)),
                ('fc2', nn.Linear(512, num_classes)),
                ('output', nn.LogSoftmax(dim=1))]))               
                
            model.classifier = classifier
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            print("Architecture not recognized.")
        return model



    def preprocess_image(self, image_path):
        
        image_pil = Image.open(image_path)
        image_pil = image_pil.convert("RGB")

        testing_transforms = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])  # Define your desired preprocessing transforms
        
        
        
        process_image = testing_transforms( image_pil)

        return process_image



    def predict(self, model, image_path, output_image_path, num_gpus):
        
        input_image = self.preprocess_image(image_path)
        # num_gpus = check_gpu_cuda()
        device = torch.device("cuda" if num_gpus > 0 else "cpu")
        model = model.to(device)    
        
        
        if num_gpus == 0:
            device = torch.device("cpu")   
            test_image_tensor = input_image.view(1, 3, 224, 224)
            input_image = test_image_tensor.to(device)
            
            
        else:
            device = torch.device("cuda")
            test_image_tensor = input_image.view(1, 3, 224, 224).cuda()
            input_image = test_image_tensor.to(device)
            
            

            test_image_tensor = input_image.view(1, 3, 224, 224).cuda()
            input_image = test_image_tensor.to('cuda')
    
        with torch.no_grad():
            model.eval()
            output = model(input_image)

            probabilities = torch.exp(output)[0]
            # print(probabilities)
        
        class_labels = ['Earthquake', 'Fire', 'Flood', 'Normal']
        class_text = list(zip(class_labels, probabilities))
            
            
        predicted_class = torch.argmax(probabilities).item()
        class_labels = ['Earthquake', 'Fire', 'Flood', 'Normal']
        class_text = list(zip(class_labels, probabilities))
        frame = cv2.imread(image_path)
        text_height = len(class_text) * 40 + 20
        text_width = 500
        text_position = (10, frame.shape[0] - text_height - 10)
        overlay = frame.copy()
        cv2.rectangle(overlay, (text_position[0], text_position[1]),
                    (text_position[0] + text_width, text_position[1] + text_height), (0, 0, 0), -1)
        cv2.rectangle(overlay, (text_position[0], text_position[1]),
                    (text_position[0] + text_width + 130, text_position[1] + text_height), (0, 0, 0), -1)
        alpha = 0.5
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        y = text_position[1] + text_height - 20
        max_prob = max([prob for _, prob in class_text])
        bar_width = 100
        bar_height = 20
        font_scale = 1.5
        
        for class_name, probability in class_text:
            color = (0, 255, 0) if class_name == class_labels[torch.argmax(probabilities)] else (0, 0, 255)
            text = f"{class_name}"
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
            cv2.putText(frame, text, (text_position[0] + 10, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)
            bar_length = int(probability / max_prob * bar_width)
            bar_start = (text_position[0] + 300, y - text_height // 2 - 5)
            bar_end = (bar_start[0] + bar_length, bar_start[1] + bar_height)
            cv2.rectangle(frame, bar_start, bar_end, color, -1)
            confidence_text = f"{probability * 100:.2f}%"
            (text_width, text_height), _ = cv2.getTextSize(confidence_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
            confidence_text_position = (430, y + text_height // 2 - 10)
            cv2.putText(frame, confidence_text, confidence_text_position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)
            y -= 40
        cv2.imwrite(output_image_path, frame)
        
            
        # print(f"Probabilities for image {os.path.basename(image_path)}:")
        result_probabilities = {}
        for class_name, probability in class_text:
            prob = probability.item() * 100
            if class_name != "Normal":
                result_probabilities[class_name] = round(prob, 2)

        # save results to the database
        frameId = database.queries.saveFrame(self.sessionId, output_image_path) # TODO dynamic session ID
        max_value = max(result_probabilities.values())
        if(max_value > 5):
            telemetry = database.queries.getDroneLatestTelemetry(self.droneId)  # TODO: dynamic drone ID # get latest drone telemetry
            # TODO dynamic session ID
            database.queries.saveDetectedDisaster(telemetry[0], telemetry[1], result_probabilities["Earthquake"], result_probabilities["Fire"], result_probabilities["Flood"], self.sessionId, frameId)
        
        return predicted_class





    #%%
    def start(self):
        root_dir = r"/app/Disaster_Classification"
        model_dir = os.path.join(root_dir, "efficientnet_model.pth")
        input_folder = os.path.join(root_dir, "images")
        output_folder = os.path.join("/media", f"disaster_classification_session{self.sessionId}_{self.droneName}")
        num_gpus = self.check_gpu_cuda()
        model = self.load_checkpoint(model_dir, num_gpus)
        os.makedirs(output_folder, exist_ok=True)

        startTime = time.time()
        detectionInterval = 1 / int(os.environ.get("COMPUTER_VISION_FPS")) # run detection X frames per second
        nextDetectionTime = startTime + detectionInterval       
        frameCounter = 0
        while not self.stop:
            currentTime = time.time()
            if currentTime >= nextDetectionTime:
                frameCounter+=1
                nextDetectionTime += detectionInterval        

                latestLiveStreamFrame = database.queries.getLatestLiveStreamFrame(self.droneId)
                image_path = latestLiveStreamFrame[0]       

                # prepare frame filename
                currentDateTime = datetime.now(timezone).time()
                formattedDateTime = currentDateTime.strftime('%H-%M-%S')
                frame_name = f"disaster-frame{frameCounter:05d}_{formattedDateTime}.jpg"

                predicted_class = self.predict(model, image_path, os.path.join(output_folder, frame_name), num_gpus)
                    
        print(f"{self.droneName}: Disaster Classification detection stopped.")


    def stopDetector(self):
        self.stop = True

