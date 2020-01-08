import atexit
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import traitlets
from matplotlib import pyplot as plt


# class Camera(traitlets.HasTraits):
    
#     value = traitlets.Any()
#     width = traitlets.Integer(default_value=640)
#     height = traitlets.Integer(default_value=480)
#     format = traitlets.Unicode(default_value='bgr8')
#     running = traitlets.Bool(default_value=False)
        
#     def __init__(self, *args, **kwargs):
#         super(Camera, self).__init__(*args, **kwargs)
#         if self.format == 'bgr8':
#             # create empty array to store camera vals
#             self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
#         # Default set to not running
#         self._running = False
    
#     def _read(self):
#         """
#         Blocking call to read frame from camera
#         """ 
#         raise NotImplementedError

#     def read(self):
#         if self._running:
#             raise RuntimeError('Cannot read directly while camera is running!')
#         self.value = self._read()
#         return self.value

#     def _capture_frames(self):
#         while True:
#             if not self._running:
#                 break
#             self.value = self._read()

#     @traitlets.observe('running')
#     def _on_running(self, change):
#         if change['new'] and not change['old']:
#             self._running = True
#             self.thread = threading.Thread(target=self._capture_frames)
#             self.thread.start()
#         elif change['old'] and not change['new']:
#             # Transition from running -> not running and join the thread to kill it
#             self._running = False
#             self.thread.join()
    
# class D435i(Camera):

#     def __init__(self, *args, **kwargs):
#         super(D435i, self).__init__(*args, **kwargs)

#         self.capture_fps = traitlets.Integer(default_value=30)
#         self.capture_width = traitlets.Integer(default_value=1280)
#         self.capture_height = traitlets.Integer(default_value=720)
#         self.capture_device = traitlets.Integer(default_value=0)

#         self.pipeline = rs.pipeline()
#         self.config = rs.config()
#         # TODO Integrate this into traitlets as defined above pyrealsense doesn't like the traitlet.traitlets.Int type need more investigation
#         self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#         self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

#         self.pipeline.start(self.config)

#         try:
#             frames = self.pipeline.wait_for_frames()
#             img_depth = self._get_str(frames)
#             self.capture = cv2.VideoCapture(0)

#             re, image = self.capture.read()

#             if not re:
#                 raise RuntimeError("Could not read image from camera!")

#         except:
#             raise RuntimeError('Could not initialize camera. Please see error trace.')

#         atexit.register(self.capture.release)

#     def _get_str(self,frames):
        
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()
    
#         depth_image = np.asanyarray(depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())

#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)

#         return np.hstack((color_image, depth_colormap))

#     def _read(self):
#         re, image = self.capture.read()
#         if re:
#             image_resized = cv2.resize(image, (int(self.width), int(self.height)))
#             return image_resized
#         else:
#             raise RuntimeError('Could not resize the image!')



# def main():
#     cam = D435i()
#     cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#     print(cam.read())
#     cv2.imshow('RealSense', cam.read())
#     cv2.waitKey(0) 
# if __name__ == "__main__":
#     main()

# Camera pipelines
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

#xARM control Interface

arm = XArmAPI('192.168.1.244')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
speed = 50
arm.set_servo_angle(angle=[0, 0, -10, 0, 0], speed=speed, wait=True)


# Camera pipelines
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)


try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue
    
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)

        images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1) 

finally:

    pipeline.stop()
