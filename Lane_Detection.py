import numpy as np
import cv2
import pygame

# ---------- 1. White Lane Color Filter ----------
def color_filter(img):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    lower_white = np.array([0, 200, 0])
    upper_white = np.array([255, 255, 255])
    white_mask = cv2.inRange(hls, lower_white, upper_white)
    return cv2.bitwise_and(img, img, mask=white_mask)

# ---------- 2. Edge Detection ----------
def edge_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

# ---------- 3. Region of Interest ----------
def region_of_interest(img):
    height, width = img.shape[:2]
    mask = np.zeros_like(img)
    polygon = np.array([[
        (int(width * 0.1), height),
        (int(width * 0.9), height),
        (int(width * 0.55), int(height * 0.62)),
        (int(width * 0.45), int(height * 0.62))
    ]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(img, mask)

#------------------check offset--------------------------------
def get_lane_offset(img, lane_lines):
    height, width = img.shape[:2]
    img_center = width // 2

    if len(lane_lines) < 1:
        return None  # No lines detected

    x_positions = []
    for x1, y1, x2, y2 in lane_lines:
        if y1 > y2:
            x_positions.append(x1)
        else:
            x_positions.append(x2)

    if len(x_positions) == 1:
        # Only one line, assume typical lane width to guess center
        lane_center = x_positions[0] + (200 if x_positions[0] < img_center else -200)
    else:
        lane_center = sum(x_positions) // len(x_positions)

    offset = img_center - lane_center
    return offset


# ---------- 4. Separate and Average Lane Lines (Handle Left/Right Optional) ----------
def average_lane_lines(img, lines):
    left_lines = []
    right_lines = []
    if lines is None:
        return []

    height, width = img.shape[:2]
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < 0.5:
            continue
        if slope < 0:
            left_lines.append(line[0])
        else:
            right_lines.append(line[0])

    def fit_line(points):
        if len(points) == 0:
            return None
        x_coords = [x1 for x1, y1, x2, y2 in points] + [x2 for x1, y1, x2, y2 in points]
        y_coords = [y1 for x1, y1, x2, y2 in points] + [y2 for x1, y1, x2, y2 in points]
        poly = np.polyfit(y_coords, x_coords, deg=1)
        y1, y2 = height, int(height * 0.6)
        x1, x2 = int(np.polyval(poly, y1)), int(np.polyval(poly, y2))
        return [[x1, y1, x2, y2]]

    result_lines = []
    left_avg = fit_line(left_lines)
    right_avg = fit_line(right_lines)

    if left_avg:
        result_lines.extend(left_avg)
    if right_avg:
        result_lines.extend(right_avg)

    return result_lines

# ---------- 5. Hough Transform ----------
def detect_lines(img):
    return cv2.HoughLinesP(img, 1, np.pi / 180, threshold=60, minLineLength=40, maxLineGap=50)

# ---------- 6. Draw Detected Lane Lines ----------
def draw_lines(img, lines, color=(0, 255, 0), thickness=5):
    line_img = np.zeros_like(img)
    if lines:
        for x1, y1, x2, y2 in lines:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    return cv2.addWeighted(img, 0.8, line_img, 1, 1)

# ---------- 7. Full Pipeline ----------
def detect_lanes_pipeline(img):
    white = color_filter(img)
    edges = edge_detection(white)
    roi = region_of_interest(edges)
    raw_lines = detect_lines(roi)
    averaged_lines = average_lane_lines(img, raw_lines)
    lane_img =  draw_lines(img, averaged_lines)

    offset = get_lane_offset(lane_img, averaged_lines)

    return lane_img, offset

# ---------- 8. Carla Integration ----------
def process_image_lane(image):
    # img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    # img_array = img_array.reshape((image.height, image.width, 4))
    # rgb_image = img_array[:, :, :3][:, :, ::-1]

    lane_img, offset = detect_lanes_pipeline(image)

    # Visual correction indicator
    if offset is not None:
        direction = "Left" if offset > 0 else "Right"
        correction = f"Offset: {abs(offset)} px {direction}"
        cv2.putText(lane_img, correction, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    frame = cv2.cvtColor(lane_img, cv2.COLOR_BGR2RGB)
    return pygame.surfarray.make_surface(frame.swapaxes(0, 1))

