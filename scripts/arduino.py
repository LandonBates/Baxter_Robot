#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


g_node = None
g_ser = None
currentPan =0
currentTilt = 0


def startup(g_ser):
    global g_node
    global currentPan
    global currentTilt
    g_ser.write("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n".encode())
    g_ser.write("sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss\n".encode())	
    currentPan = -1.6
    currentTilt=0

	
def sub_callbackPan(msg):
    global g_node
    global g_ser
    global currentPan
    global currentTilt
    g_node.get_logger().info('Panning To:"%f"'%msg.data)
    command = ""
    while (msg.data>currentPan):
        if((msg.data -currentPan)> .5):
            command+='A'
            currentPan += .5
        else:
            command +='a'
            currentPan += .01

    while (msg.data<currentPan):
        if((currentPan - msg.data)> 0.5):
            command+='D'
            currentPan -= .5
        else:
            command+='d'
            currentPan -= .01

    g_ser.write(command.encode())
def sub_callbackTilt(msg):
    global g_node
    global g_ser
    global currentPan
    global currentTilt
    g_node.get_logger().info('Tillting to:"%f"'%msg.data)
    command = ""

    while(msg.data>currentTilt):
        if((msg.data-currentTilt)>.5):
            command+='W'
            currentTilt += .5
        else:
            command+='w'
            currentTilt += .01

    while(msg.data<currentTilt):
        if((currentTilt-msg.data)>0.5):
            command+='S'
            currentTilt -= .5
        else:
            command+='s'
            currentTilt -= .01
    g_ser.write(command.encode())

def main(args=None):
    global g_node
    global g_ser
    global currentPan
    global currentTilt
    rclpy.init(args=args)
    g_node = rclpy.create_node('arduino_publisher')

    publisher = g_node.create_publisher(JointState, 'joint_states', 10)
    subPan = g_node.create_subscription(Float64, 'pan_cmd',sub_callbackPan, 10)
    subTilt = g_node.create_subscription(Float64, 'tilt_cmd',sub_callbackTilt, 10)

    joints= JointState()
    joints.name =["pan_joint", "tilt_joint"]
    joints.position = [0.0, 0.0]
    publisher.publish(joints)
	

    i = 0
    #g_node.get_logger().info('Starting Serial')
    g_ser=serial.Serial('/dev/ttyACM0', 115200, timeout=100)
    # line=g_ser.readline(10)
    # vals=line.decode().split(',')
    # g_node.get_logger().info('got a line from the arduino?')
    # while(len(vals) < 1 ):
    #     line=g_ser.readline(10)
    #     vals=line.decode().split(',')
    #     g_node.get_logger().info('waiting for arduino to respond')
    #g_node.get_logger().info('running startup function')
    startup(g_ser)
    g_node.get_logger().info('finished startup starting arduino loop')
    while rclpy.ok():
        line=g_ser.readline(3)
        
        vals=line.decode().split(',')
        g_node.get_logger().info('got a line from the arduino?')
        if(len(vals) >= 2):
            print("Got ",len(vals)," vals from ",line)
            LimitSwitchRight= float(vals[0])
            LimitSwitchNod= float(vals[1])
            if(LimitSwitchNod==1):
                currentTilt=0
            if(LimitSwitchRight == 1):
                currentPan = 1.6
        	#continue
        g_node.get_logger().info('setting current pan and tilt')
        joints.position[0] = currentPan
        joints.position[1] = currentTilt
    
        	
        g_node.get_logger().info('Serial Rx: %s'%str(line))
        publisher.publish(joints)
        rclpy.spin_once(g_node, timeout_sec=0)

    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
