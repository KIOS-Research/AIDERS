# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 12:11:08 2023

@author: dshian01
"""

import cv2
import torch
from torchvision import models
from torchvision import transforms
import PIL.Image
import torch.nn as nn
from collections import OrderedDict
import os


#%%

def check_gpu_cuda():
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
        
def load_checkpoint(filepath, num_gpus):
    
    
    if num_gpus == 0:
        
        checkpoint = torch.load(filepath, map_location=torch.device('cpu'))
    
    else:
        
        checkpoint = torch.load(filepath)
        
        
    num_classes = 4

    print(checkpoint['arch'])


    if checkpoint['arch'] == 'efficientnet':
        model = models.efficientnet_b0(pretrained=True)
        
        for param in model.parameters():
            param.requires_grad = False
        
        model.class_to_idx = checkpoint['class_to_idx']
        
        in_features = model.classifier[-1].in_features
        
        classifier = nn.Sequential(OrderedDict([('fc1', nn.Linear(in_features, 512)),
                                                ('relu', nn.ReLU()),
                                                ('drop', nn.Dropout(p=0.5)),
                                                ('fc2', nn.Linear(512, num_classes)),
                                                ('output', nn.LogSoftmax(dim=1))]))
        

        model.classifier = classifier
        
        model.load_state_dict(checkpoint['model_state_dict'])
        
   

    else:
        print("Architecture not recognized.")



    
    return model

def video_prediction(model_path , video_path, output_path):
    
    num_gpus = check_gpu_cuda()
    
    model = load_checkpoint(model_path, num_gpus)
    
    if num_gpus == 0:
        
        device = torch.device("cpu")
    
    else:
        device = torch.device("cuda")
        
    model.to(device)
    
    model.eval()  # Set the model to evaluation mode

    
    text_color = (255, 255, 255)  # White color for text
    box_color = (0, 0, 0)  # Black color for box
    right_class_color = (0, 0, 255)  # Blue color for the correct class
    text_position = (10, 10)  # (x, y) coordinates of the top-left corner of the text box
    position_option = 'upper_left'

    testing_transforms = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])  # Define your desired preprocessing transforms
    class_labels = ['Earthquake', 'Fire', 'Flood', 'Normal']
    
    
    cap = cv2.VideoCapture(video_path)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    output_video = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))



    
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break
        
        # frame = frame[:, 0:(frame_width - 300)]
        
        # Preprocess the frame
        pil_frame = PIL.Image.fromarray(frame)
        preprocessed_frame = testing_transforms(pil_frame)
        preprocessed_frame = preprocessed_frame.unsqueeze(0)  # Add batch dimension

        # Perform inference
        with torch.no_grad():
            output = model(preprocessed_frame)
            probabilities = torch.exp(output)[0]

        # Convert probabilities to text
        class_text = []
        for i in range(len(probabilities)):
            probability = probabilities[i].item()
            class_text.append((class_labels[i], probability))

        # Draw the transparent black box
        text_height = len(class_text) * 40 + 20  # Adjust the height of the box
        text_width = 500  # Adjust the width of the box
        if position_option == 'upper_left':
            text_position = (10, 10)
        elif position_option == 'upper':
            text_position = (frame.shape[1] // 2 - text_width // 2, 10)
        elif position_option == 'upper_right':
            text_position = (frame.shape[1] - text_width - 10, 10)
        elif position_option == 'bottom_left':
            text_position = (10, frame.shape[0] - text_height - 10)
        elif position_option == 'bottom_right':
            text_position = (frame.shape[1] - text_width - 10, frame.shape[0] - text_height - 10)
        else:
            raise ValueError('Invalid position option')

        overlay = frame.copy()
        cv2.rectangle(overlay, (text_position[0], text_position[1]),
                      (text_position[0] + text_width, text_position[1] + text_height), box_color, -1)
        
        cv2.rectangle(overlay, (text_position[0], text_position[1]),
              (text_position[0] +  text_width + 130, text_position[1] + text_height), box_color, -1)


        alpha = 0.5
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

        # Draw the class labels, probabilities, and proportional bar chart on the frame
        y = text_position[1] + text_height - 20  # Starting y-coordinate for text
        max_prob = max([prob for _, prob in class_text])
        bar_width = 100
        bar_height = 20  # Adjust the height of the bar (thinner bar)
        font_scale = 1.5  # Increase the font size
        for class_name, probability in class_text:
            if class_name == class_labels[torch.argmax(probabilities)]:
                color = right_class_color  # Use specified text color for correct class
            else:
                color = text_color
            text = f"{class_name}"
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
            cv2.putText(frame, text, (text_position[0] + 10, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)
        
            bar_length = int(probability / max_prob * bar_width)
            bar_start = (text_position[0] + 300, y - text_height // 2 + -5)  # Adjust the position of the bar
            bar_end = (bar_start[0] + bar_length, bar_start[1] + bar_height)  # Adjust the height of the bar
            cv2.rectangle(frame, bar_start, bar_end, color, -1)
            
            confidence_text = f"{probability * 100:.2f}%"  # Confidence score as a percentage
            (text_width, text_height), _ = cv2.getTextSize(confidence_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
            confidence_text_position = (430, y + text_height // 2 - 10)
            cv2.putText(frame, confidence_text, confidence_text_position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)
        
            y -= 40  # Increase the spacing between text
        
        # Display the frame with class labels, probabilities, and proportional bar charts
        cv2.imshow('Video', frame)
        output_video.write(frame)  # Write frame to the output video

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object, release the output video, and close windows
    cap.release()
    output_video.release()
    cv2.destroyAllWindows()



#%%
def main():

    root_dir = r"C:\Users\dshian01\OneDrive - University of Cyprus\Desktop\Server_Scripts\LightWeightModels\AIDERS_platform"
    video_dir = os.path.join(root_dir, "demo3.mp4")
    model_dir = os.path.join(root_dir, "efficientnet_model.pth")
    output_dir = os.path.join(root_dir, "pred.mp4")

    video_prediction(model_dir, video_dir,output_dir)

if __name__ == "__main__":
    main()

