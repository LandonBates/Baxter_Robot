#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum

class State(Enum):
  PAN=0
  TRACK=1

class executive(Node):
  def __init__(self):
    self.pub_pan_cmd = self.create_publisher(float, 'pan_cmd', 10)
    self.pub_tilt_cmd = self.create_publisher(float, 'tilt_cmd', 10)
    self.sub_detections = self.create_subscription(AprilTagDetectionArray,
      'detections',self.detections_callback,10)
    
  def detections_callback(self, msg):
    from_frame = 'chin'
    for d in msg.detections:
      tag_id = d.id
      to_frame = "tag36h11:0"
      try:
        t = self.tf_buffer.lookup_transform(
          from_frame,
          to_frame,
          rclpy.time.Time())
      except TransformException as ex:
        self.get_logger().info(
          f'Could not transform {to_frame} to {from_frame}: (ex)')
        return
      self.get_logger().info("detected apriltag at x={:f} y={:f}".format(
        t.transform.translation.x,t.transform.translation.y))

def main(args=None):

  rclpy.init(args=args)
  executive_node = executive()
  rclpy.spin(executive_node)
  rclpy.shutdown()

if __name__=="__main__":
  main()
