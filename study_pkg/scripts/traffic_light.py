import cv2
import numpy as np
import pyrealsense2 as rs
from realsense import Realsense
from detection import *


realsense = Realsense()


realsense.start_stream()

resize_factor = 1

circle_params = dict(dp=1, minDist=100, param1=30, param2=50, minRadius=19, maxRadius=51)

while True:
    color_frame, depth_frame = realsense.get_frames()

    # if depth_frame.size == 0 or color_frame.size == 0:
    #     continue


    detected_light, circle = detect_traffic_light_color(color_frame, circle_params)

    if detected_light and circle is not None:
        x, y, r = circle
        color_frame = draw_traffic_light_circle(color_frame, detected_light, circle)


        distance = depth_frame[int(y), int(x)] * realsense.depth_scale
        if distance > 0:
            command = calculate_braking_command(detected_light, distance)
            # print(f"Distance to traffic light: {distance:.2f} meters")
            msg2=distance
            color_frame = draw_command_on_frame(color_frame, command)
            cv2.putText(color_frame, f"Distance: {distance:.2f} meters", (20, 90), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)



    depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.12), cv2.COLORMAP_JET)


    depth_colored_resized = cv2.resize(depth_colored, (
        int(depth_colored.shape[1] * resize_factor), int(depth_colored.shape[0] * resize_factor)))


    color_frame_resized = cv2.resize(color_frame, (
        int(color_frame.shape[1] * resize_factor), int(color_frame.shape[0] * resize_factor)))


    combined_frame = cv2.hconcat([color_frame_resized, depth_colored_resized])

    cv2.imshow("Detect and Depth", combined_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q') or key == ord('Q'):
        break

realsense.stop_stream()
cv2.destroyAllWindows()