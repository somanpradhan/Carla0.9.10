import numpy as np
import cv2
import pygame

def color_filter(img):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    lower_white = np.array([0, 200, 0])
    upper_white = np.array([255, 255, 255])
    white_mask = cv2.inRange(hls, lower_white, upper_white)
    return cv2.bitwise_and(img, img, mask=white_mask)

def edge_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

def region_of_interest(img):
    height, width = img.shape[:2]
    mask = np.zeros_like(img)
    # Wider polygon to accommodate curves
    polygon = np.array([[
        (int(width * 0.05), height),
        (int(width * 0.95), height),
        (int(width * 0.65), int(height * 0.5)),
        (int(width * 0.35), int(height * 0.5))
    ]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(img, mask)

def sliding_window_polyfit(img):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(img[img.shape[0]//2:,:], axis=0)
    
    # Find the peak of the left and right halves
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    # Choose the number of sliding windows
    nwindows = 9
    window_height = np.int(img.shape[0]//nwindows)
    
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    return left_fit, right_fit

def draw_polyfit(img, left_fit, right_fit):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    # Create an image to draw the lines on
    warp_zero = np.zeros_like(img).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    
    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
    
    # Draw the lane lines
    for i in range(len(ploty)-1):
        cv2.line(color_warp, (int(left_fitx[i]), int(ploty[i])), 
                 (int(left_fitx[i+1]), int(ploty[i+1])), (255,0,0), 5)
        cv2.line(color_warp, (int(right_fitx[i]), int(ploty[i])), 
                 (int(right_fitx[i+1]), int(ploty[i+1])), (0,0,255), 5)
    
    # Combine the result with the original image
    result = cv2.addWeighted(img, 1, color_warp, 0.3, 0)
    return result

def get_lane_offset(img, left_fit, right_fit):
    height = img.shape[0]
    img_center = img.shape[1] // 2
    
    # Calculate x positions at bottom of image
    left_x = left_fit[0]*height**2 + left_fit[1]*height + left_fit[2]
    right_x = right_fit[0]*height**2 + right_fit[1]*height + right_fit[2]
    
    lane_center = (left_x + right_x) // 2
    offset = img_center - lane_center
    return offset

def detect_lanes_pipeline(img):
    white = color_filter(img)
    edges = edge_detection(white)
    roi = region_of_interest(edges)
    
    try:
        left_fit, right_fit = sliding_window_polyfit(roi)
        lane_img = draw_polyfit(img, left_fit, right_fit)
        offset = get_lane_offset(img, left_fit, right_fit)
    except:
        # Fallback to straight line detection if polynomial fit fails
        raw_lines = cv2.HoughLinesP(roi, 1, np.pi/180, 60, minLineLength=40, maxLineGap=50)
        averaged_lines = average_lane_lines(img, raw_lines)
        lane_img = draw_lines(img, averaged_lines)
        offset = get_lane_offset(img, averaged_lines)
    
    return lane_img, offset

def process_image_lane(image):
    img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    img_array = img_array.reshape((image.height, image.width, 4))
    rgb_image = img_array[:, :, :3][:, :, ::-1]

    lane_img, offset = detect_lanes_pipeline(rgb_image)

    if offset is not None:
        direction = "Left" if offset > 0 else "Right"
        correction = f"Offset: {abs(offset):.1f} px {direction}"
        cv2.putText(lane_img, correction, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    frame = cv2.cvtColor(lane_img, cv2.COLOR_BGR2RGB)
    return pygame.surfarray.make_surface(frame.swapaxes(0, 1))


