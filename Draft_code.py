import numpy as np
import cv2
import pygame

def detect_edges(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 70, 150)  # Tweak these if needed
    return edges

def region_of_interest(img):
    height, width = img.shape
    mask = np.zeros_like(img)

    polygon = np.array([[
        (int(width * 0.2), height),
        (int(width * 0.8), height),
        (int(width * 0.6), int(height * 0.58)),
        (int(width * 0.4), int(height * 0.58))
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(img, mask)

def detect_lines(img):
    lines = cv2.HoughLinesP(
        img,
        rho=1,
        theta=np.pi / 180,
        threshold=60,
        minLineLength=50,
        maxLineGap=150
    )
    return lines

def filter_lane_lines(lines, img_height):
    if lines is None:
        return []

    filtered = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        length = np.hypot(x2 - x1, y2 - y1)

        # Conditions to keep good lane lines:
        if 25 < abs(angle) < 75 and length > 40:
            # Skip lines too close to the bottom (arrows)
            if y1 < img_height - 50 and y2 < img_height - 50:
                filtered.append(line)
    return filtered


def draw_lines(img, lines, color=(0, 255, 0), thickness=4):
    line_img = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    return cv2.addWeighted(img, 0.8, line_img, 1, 1)

def detect_lanes_pipeline(rgb_image):
    edges = detect_edges(rgb_image)
    cropped_edges = region_of_interest(edges)
    raw_lines = detect_lines(cropped_edges)
    good_lines = filter_lane_lines(raw_lines, rgb_image.shape[0])
    lane_image = draw_lines(rgb_image, good_lines)
    return lane_image


def process_image_lane(image):
    # Convert to RGB NumPy array
    img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    img_array = img_array.reshape((image.height, image.width, 4))  # BGRA
    rgb_image = img_array[:, :, :3][:, :, ::-1]  # Convert to RGB

    # Lane detection pipeline
    result = detect_lanes_pipeline(rgb_image)

    frame = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)

    return pygame.surfarray.make_surface(frame.swapaxes(0, 1))