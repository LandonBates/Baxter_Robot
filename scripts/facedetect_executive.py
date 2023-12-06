#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class executive(Node):
  def __init__(self):
    super().__init__('executive')
    self.pan_msg = Float64()
    self.tilt_msg = Float64()
    self.current_pan = 0
    self.current_tilt = 0
    self.leftright = True
    self.pub_pan_cmd = self.create_publisher(Float64, 'pan_cmd', 10)
    self.pub_tilt_cmd = self.create_publisher(Float64, 'tilt_cmd', 10)
    self.sub_detections = self.create_subscription(Float64MultiArray,
      'face_detection',self.face_detect_callback,10)
    self.sub_jointstates = self.create_subscription(JointState, 'joint_states',
      self.jointstates_callback,10)
    self.pub_tilt_cmd.publish(self.tilt_msg)
    
  def jointstates_callback(self,msg):
    self.current_pan = msg.position[0]
    self.current_tilt = msg.position[1]
    
  def face_detect_callback(self, msg):

      face_x = (290.0 - msg.data[0])*(0.8/580.0)
      face_y = (190.0 - msg.data[1])*(0.5/380.0)
      self.get_logger().info("%f %f" % (face_x, face_y))
        
      if (face_x > 0.025) or (face_x < -0.025):
        self.pan_msg.data =  self.current_pan - face_x
      else:
        self.pan_msg.data = 0.0
      self.pub_pan_cmd.publish(self.pan_msg)

      #if (face_y > 0.025) or (face_y < -0.025):
      #  self.tilt_msg.data = face_y 
      #else:
      #  self.tilt_msg.data = 0
      #self.pub_tilt_cmd.publish(self.tilt_msg)
    
    #if len(msg.detections) == 0:
      #self.get_logger().info("no apriltag detected: searching...")
      
      #if (self.current_pan >= 1.6) or (self.current_pan <= -1.6):
      #  self.leftright = not self.leftright
      
      #if (self.leftright):
      #  self.pan_msg.data = self.current_pan + 0.25
      #else:
      #  self.pan_msg.data = self.current_pan - 0.25
        
      #self.pub_pan_cmd.publish(self.pan_msg)
      
def main(args=None):

  rclpy.init(args=args)
  executive_node = executive()
  rclpy.spin(executive_node)
  rclpy.shutdown()

if __name__=="__main__":
  main()
