#We gonna use this code to detect the object in the camera sensor
import numpy as np
import cv2
import pygame
from ultralytics import YOLO
from Lane_Detection import process_image_lane


# Load the YOLOv8 model
#model = YOLO("C:\\Users\\acer\\carla\\carla\\Build\\UE4Carla\\0.9.10-dirty\\WindowsNoEditor\\PythonAPI\\OurCode\\train2\\weights\\best.pt").to("cuda")
#model = YOLO("F:\\augmented\\augmented\\runs\\detect\\train6\\weights\\best.pt").to("cuda")
#model = YOLO("F:\\augmented\\augmented\\best_model.pt").to("cuda")
#model = YOLO("yolov8n.pt").to("cuda")
model = YOLO("G:\\Training\\Training\\runs\\detect\\train8\\weights\\best.pt").to("cuda")  # Load the YOLOv8 model (Replace with your trained model path)
#@staticmethod
def parse_image(image):
    state = True
    if image is None:
        return None
    
    # Convert raw image data to a numpy array
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4)) # RGBA format

    # Convert RGBA to BGR for OpenCV
    frame = array[:, :, :3].copy()

    #frame_resized = cv2.resize(frame, (480, 320))

     # ✅ Convert image to Tensor (Move to GPU)

    # Run YOLOv8 detection (direct tensor conversion for speed)
    #results = model(frame_resized, verbose=False)[0]
    results = model(frame, verbose=False)[0]
    labels = []
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])  
        conf = float(box.conf[0])
        label = model.names[int(box.cls[0])]

        # Scale bounding boxes back to original resolution
        #x1 = int(x1 * frame.shape[1] / 480)
        #x2 = int(x2 * frame.shape[1] / 480)
        #y1 = int(y1 * frame.shape[0] / 320)
        #y2 = int(y2 * frame.shape[0] / 320)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if conf >= 0.85:
            labels.append(label)

    # Convert back to Pygame format
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #frame = frame[:, :, ::-1]
    #frame = cv2.transpose(frame)
    #frame = cv2.flip(frame, 0)
    #return pygame.surfarray.make_surface(np.rot90(frame))
    if not(state):
        return pygame.surfarray.make_surface(frame.swapaxes(0, 1)), labels
    surface = process_image_lane(frame)
    return surface, labels
