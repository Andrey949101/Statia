import cv2
import numpy as np
import pyrealsense2 as rs

class Realsense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Запуск потоков
        profile = self.pipeline.start(self.config)

        # Получение масштаба глубины
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def start_stream(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass

        self.pipeline.start(self.config)

    def get_frames(self):
        # Получение кадров
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        # Преобразование кадров
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    def stop_stream(self):
        # Остановка потока
        self.pipeline.stop()