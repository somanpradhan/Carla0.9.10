#We gonna use this code to detect the object in the camera sensor
import numpy as np
import cv2
import pygame
from ultralytics import YOLO
from Lane_Detection import process_image_lane


# Define Area of Interest (AOI) in image coordinates
AOI_LEFT = 250
AOI_RIGHT = 550
AOI_TOP = 300
AOI_BOTTOM = 600

# Load the YOLOv8 model
# model = YOLO("C:\\Users\\acer\\Documents\\runs\\runs\\detect\\train2\\weights\\best.pt").to("cuda")  # Load the YOLOv8 model (Replace with your trained model path) #model = YOLO("yolov8n.pt").to("cuda") # 
model = YOLO("best444.pt").to("cuda")  # Load the YOLOv8 model (Replace with your trained model path) #model = YOLO("yolov8n.pt").to("cuda") 
def parse_image(image):
    state = False
    if image is None:
        return None
    
    # Convert raw image data to a numpy array
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4)) # RGBA format

    # Convert RGBA to BGR for OpenCV
    frame = array[:, :, :3].copy()


     #  Convert image to Tensor (Move to GPU)

    results = model(frame, verbose=False)[0]
    labels = []
    confs = []
    image_center_x = image.width / 2
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])  
        conf = float(box.conf[0])
        label = model.names[int(box.cls[0])]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if conf >= 0.75:
            labels.append(label)
            confs.append(conf)
            if label == "pedestrian":
                # Bounding box
                bbox_left = x1
                bbox_top = y1
                bbox_right = x2
                bbox_bottom = y2

        # AOI rectangle
        # (define these globally or earlier in your code)
        # AOI_LEFT, AOI_TOP, AOI_RIGHT, AOI_BOTTOM

        # Check if bounding box intersects with AOI
                inter_left = max(x1, AOI_LEFT)
                inter_top = max(y1, AOI_TOP)
                inter_right = min(x2, AOI_RIGHT)
                inter_bottom = min(y2, AOI_BOTTOM)

                inter_width = max(0, inter_right - inter_left)
                inter_height = max(0, inter_bottom - inter_top)
                intersection_area = inter_width * inter_height

                # Total area of bounding box
                bbox_area = (x2 - x1) * (y2 - y1)
                in_aoi = False

                if bbox_area != 0:
                    overlap_ratio = intersection_area / bbox_area
                    in_aoi = overlap_ratio >= 0.3

                bbox_height = y2 - y1
                bbox_width = x2 - x1

                #AOI Check
                #Distance check
                close_enough = bbox_height >= 180 or bbox_width >= 50
            # Heuristic: close if tall box, centered if near middle of screen
                if in_aoi and close_enough:
                    state = True


    # Convert back to Pygame format
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # if not(state):
    #     return pygame.surfarray.make_surface(frame.swapaxes(0, 1)), labels
    # surface = process_image_lane(frame)
    # return surface,state, labels
   
    return pygame.surfarray.make_surface(frame.swapaxes(0, 1)), state, [labels, confs]
