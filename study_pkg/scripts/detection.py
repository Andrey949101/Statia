import cv2
import numpy as np


def detect_traffic_light_color(color_frame, circle_params):
    roi_x = 340
    roi_y = 0
    roi_width = 300
    roi_height = 400

    cv2.rectangle(color_frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (0, 255, 0), 2)
    roi = color_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

    red_high_lower = np.array([0, 55, 180], dtype="uint8")
    red_high_upper = np.array([10, 255, 250], dtype="uint8")

    yellow_lower = np.array([13, 55, 180], dtype="uint8")
    yellow_upper = np.array([25, 255, 255], dtype="uint8")

    green_lower = np.array([40, 85, 180], dtype="uint8")
    green_upper = np.array([91, 255, 255], dtype="uint8")

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv, red_high_lower, red_high_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_sum = np.sum(red_mask)
    yellow_sum = np.sum(yellow_mask)
    green_sum = np.sum(green_mask)

    if red_sum > yellow_sum and red_sum > green_sum:
        color = 'red'
    elif yellow_sum > red_sum and yellow_sum > green_sum:
        color = 'yellow'
    elif green_sum > red_sum and green_sum > yellow_sum:
        color = 'green'
    else:
        color = None

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, **circle_params)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        max_radius = np.max(circles[0, :, 2])
        max_circle_index = np.argmax(circles[0, :, 2])
        max_circle = circles[0, max_circle_index, :]
        x, y, r = max_circle
        x += roi_x
        y += roi_y

        circle_roi = roi[y - r - roi_y:y + r - roi_y, x - r - roi_x:x + r - roi_x]
        if circle_roi.any():
            hsv_roi = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2HSV)

            red_roi_mask = cv2.inRange(hsv_roi, red_high_lower, red_high_upper)
            yellow_roi_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
            green_roi_mask = cv2.inRange(hsv_roi, green_lower, green_upper)

            red_roi_sum = np.sum(red_roi_mask)
            yellow_roi_sum = np.sum(yellow_roi_mask)
            green_roi_sum = np.sum(green_roi_mask)

            if red_roi_sum > yellow_roi_sum and red_roi_sum > green_roi_sum:
                color = 'red'
            elif yellow_roi_sum > red_roi_sum and yellow_roi_sum > green_roi_sum:
                color = 'yellow'
            elif green_roi_sum > red_roi_sum and green_roi_sum > yellow_roi_sum:
                color = 'green'
            else:
                color = None

        return color, (x, y, r)
    else:
        return None, None


def get_color_from_circle(roi, circle):
    x, y, r = circle

    circle_roi = roi[y - r:y + r, x - r:x + r]
    if not circle_roi.any():
        return None

    hsv_roi = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2HSV)

    red_high_lower = np.array([0, 55, 180], dtype="uint8")
    red_high_upper = np.array([10, 255, 250], dtype="uint8")

    yellow_lower = np.array([13, 55, 180], dtype="uint8")
    yellow_upper = np.array([25, 255, 255], dtype="uint8")

    green_lower = np.array([40, 85, 180], dtype="uint8")
    green_upper = np.array([91, 255, 255], dtype="uint8")

    red_roi_mask = cv2.inRange(hsv_roi, red_high_lower, red_high_upper)
    yellow_roi_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
    green_roi_mask = cv2.inRange(hsv_roi, green_lower, green_upper)

    red_roi_sum = np.sum(red_roi_mask)
    yellow_roi_sum = np.sum(yellow_roi_mask)
    green_roi_sum = np.sum(green_roi_mask)

    if red_roi_sum > yellow_roi_sum and red_roi_sum > green_roi_sum:
        color = 'red'
    elif yellow_roi_sum > red_roi_sum and yellow_roi_sum > green_roi_sum:
        color = 'yellow'
    elif green_roi_sum > red_roi_sum and green_roi_sum > yellow_roi_sum:
        color = 'green'
    else:
        color = None

    return color

def detect_light_color(roi, circles):
    detected_colors = []
    for circle in circles:
        detected_color = get_color_from_circle(roi, (*circle,))
        detected_colors.append((detected_color, circle))

    return detected_colors

def draw_traffic_light_circle(color_frame, detected_light, circle):
    x, y, r = circle
    if detected_light == 'red':
        circle_color = (0, 0, 255)
    elif detected_light == 'yellow':
        circle_color = (0, 100, 255)
    elif detected_light == 'green':
        circle_color = (0, 255, 0)
    else:
        circle_color = (255, 255, 255)

    cv2.circle(color_frame, (x, y), r, circle_color, 2)
    cv2.circle(color_frame, (x, y), 2, (0, 0, 255), 3)
    cv2.putText(color_frame, f"Color: {detected_light}", (20, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)

    return color_frame

def calculate_braking_command(light_color, distance):
    yellow_light_duration = 4
    yellow_braking_distance = 1.2
    red_braking_distance = 0.5
    red_maintain_speed_distance = 1.4

    # if light_color == "green":
    #     if distance > yellow_braking_distance:
    #         return "Gas"
    #     else:
    #         return "maintain speed"
    # elif light_color == "yellow":
    #     deceleration_time = yellow_light_duration * (distance / yellow_braking_distance)
    #     if distance > yellow_braking_distance:
    #         return "maintain speed"
    #     else:
    #         return "soft brake"
    # elif light_color == "red":
    #     if distance > red_maintain_speed_distance:
    #         return "maintain speed"
    #     elif red_maintain_speed_distance >= distance > red_braking_distance:
    #         return "soft brake"
    #     else:
    #         return "Brake"
    # else:
    #     return "unknown"

def draw_command_on_frame(color_frame, command):
    if command == "Gas":
        command_text = "Gas"
        color = (0, 255, 0)
    elif command == "maintain speed":
        command_text = "Maintain Speed"
        color = (255, 255, 255)
    elif command == "soft brake":
        command_text = "Soft Brake"
        color = (0, 255, 255)
    elif command == "Brake":
        command_text = "Brake"
        color = (0, 0, 255)
    else:
        command_text = "Unknown"
        color = (255, 0, 0)

    cv2.putText(color_frame, command_text, (20, 130), cv2.FONT_HERSHEY_DUPLEX, 1, color, 1, cv2.LINE_AA)
    return color_frame
