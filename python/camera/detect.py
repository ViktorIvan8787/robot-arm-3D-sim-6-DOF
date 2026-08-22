# Code for camera to detect objects using YOLO [Currently only detects "cup" (there is no input right now)]

# Open camera make sure that it reads frame properly and capture the frame

import cv2
from ultralytics import YOLO

# Load the model 
model = YOLO("yolov8n.pt")

# Capture a still image from the video camera {default camera: 0} 
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()

# Check if the frame was read correctly using the ret variable

if not ret :
    raise RuntimeError("Failed to capture image")

# Otherwise we can pass the frame into the model and start detecting
cv2.imwrite("captured_frame.jpg", frame)

# Get the results of passing the captured frame into the model 
results = model("captured_frame.jpg")

# Iterate over the results and find the object that you are looking for 

target_class = "cup"

for image in results:
    for object in image.boxes:
        # get the class name of the object to compare to the target class
        class_name = model.name[int(object.cls[0])]
        # get the confidence for this object 
        confidence = float(object.conf[0])

        # check if it is the right object
        if target_class == class_name:
            # Get centre coordinates of the object
            x , y , _ , _ = object.xywh[0]
            # Print out that object has been found for test units and debugging
            print(f"{target_class} object found, (conf {confidence:.2f}) at pixel coordinates ({x:.0f}, {y:.0f})")