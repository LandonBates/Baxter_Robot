#! /usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import cv2 as cv
import argparse

class objectDetection(Node):
    def __init__(self):
        super().__init__('objectDetection')
        robot_path = get_package_share_directory('robot')

        self.face_cascade_name = robot_path+'/models/face/haarcascade_frontalface_alt.xml'

        self.face_cascade = cv.CascadeClassifier()
		
        self.camera_callback = self.create_subscription(JointState, 'joint_states', self.camera_callback,10)
        self.pub_detections = self.create_publisher(Float64MultiArray, 'face_detection', 10)
        self.face_pos = Float64MultiArray()
        self.face_pos.data = [0.0, 0.0]

        self.face_cascade.load(cv.samples.findFile(self.face_cascade_name))
		    
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv.CAP_PROP_FPS, 30)

    def camera_callback(self, msg):
        ret, frame = self.cap.read()
        if frame is not None:
            frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frame_gray = cv.equalizeHist(frame_gray)

            #-- Detect faces
            faces = self.face_cascade.detectMultiScale(frame_gray)
            for (x,y,w,h) in faces:
                center = (x + w//2, y + h//2)
                frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
                
                self.face_pos.data[0] = x
                self.face_pos.data[1] = y

                faceROI = frame_gray[y:y+h,x:x+w]
                self.pub_detections.publish(self.face_pos)
        cv.imshow('Capture - Face detection', frame)
        
def main(args=None):

    rclpy.init(args=args)
    node = objectDetection()
    cv.namedWindow("view",cv.WINDOW_NORMAL);
    cv.setWindowProperty("view",cv.WND_PROP_FULLSCREEN,cv.WINDOW_FULLSCREEN);
    cv.moveWindow("view", 1024, 600);
    cv.startWindowThread();

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
  main()
