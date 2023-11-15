#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum
from sensor_msgs.msg import JointState

class State(Enum):
  PAN=0
  TRACK=1

class executive(Node):
  def __init__(self):
    super().__init__('executive')
    self.pan_msg = Float64()
    self.current_pan = 0.0 
    self.tilt_msg = Float64()
    self.current_tilt = 0.0
    self.pub_pan_cmd = self.create_publisher(Float64, 'pan_cmd', 10)
    self.pub_tilt_cmd = self.create_publisher(Float64, 'tilt_cmd', 10)
    self.sub_detections = self.create_subscription(AprilTagDetectionArray,
      'detections',self.detections_callback,10)
    self.sub_jointstates = self.create_subscription(JointState, 'joint_states',
      self.jointstates_callback,10)
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    
  def jointstates_callback(self,msg):
    self.current_pan = msg.position[1]
    self.current_tilt = msg.position[2]
    
  def detections_callback(self, msg):
    from_frame = 'camera'
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
          f'Could not transform {to_frame} to {from_frame}')
        return
      self.april_x = t.transform.translation.x
      self.april_y = t.transform.translation.y
      self.get_logger().info("detected apriltag at x={:f} y={:f}".format(
        t.transform.translation.x,t.transform.translation.y))
        
      if (self.april_x > 0.01):
        self.pan_msg.data = self.current_pan + self.april_x 
      elif (self.april_x < -0.01):
        self.pan_msg.data = self.current_pan - self.april_x 
      self.pub_pan_cmd.publish(self.pan_msg)

      if (self.april_y > 0.01):
        self.tilt_msg.data = self.current_tilt - self.april_y 
      elif (self.april_y < -0.01):
        self.tilt_msg.data = self.current_tilt + self.april_y
      self.pub_tilt_cmd.publish(self.tilt_msg)
def main(args=None):

  rclpy.init(args=args)
  executive_node = executive()
  rclpy.spin(executive_node)
  rclpy.shutdown()

if __name__=="__main__":
  main()
