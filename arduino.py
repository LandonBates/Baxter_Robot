#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Pose

g_node = None
g_ser = None
currentRoll =0
currentPitch = 0
desiredRoll =0
desiredPitch =0

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
def startup():
	global g_node
	global g_ser
	g_ser.write("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n".encode())
	g_ser.write("sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss\n".encode())	
	currentRoll = -1.6;
	currentPitch=0;

	
def sub_callback(msg):
    global g_node
    global g_ser
    g_node.get_logger().info('I heard "%f"'%msg.pose)
    desiredRoll, desiredPitch = euler_from_quaternion(msg.pose.x, msg.pose.y, msg.pose.z, msg.pose.w)
    command = ""
    while (desiredRoll>currentRoll):
    	command.append('a')
    	currentRoll += .02
    while(desiredPitch>currentPitch):
    	command.append('w')
    	currentPitch += .02
    while (desiredRoll<currentRoll):
    	command.append('d')
    	currentRoll -= .02
    while(desiredPitch<currentPitch):
    	command.append('s')
    	currentPitch -= .02
    g_ser.write(command.encode())

def main(args=None):
    global g_node
    global g_ser
    rclpy.init(args=args)
    g_node = rclpy.create_node('arduino_publisher')

    publisher = g_node.create_publisher(Pose, 'arduino_feedback', 1)
    subscriber = g_node.create_subscription(Pose, 'arduino_command',\
            sub_callback, 1)

    msg = Pose()

    i = 0
    g_ser=serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    startup()
    while rclpy.ok():
        line=g_ser.readline(10)
        vals=line.decode().split(',')
        if(len(vals) != 3):
            print("Got ",len(vals)," vals from ",line)
            continue
        print("Got ",len(vals)," vals from ",line)
        LimitSwitchLeft= float(vals[0])
        LimitSwitchRight= float(vals[1])
        LimitSwitchNod= float(vals[2])
        if(LimitSwitchLeft==1 && LimitSwitchNod==1):
       		currentRoll = -1.6;
		currentPitch=0;
		msg.pose = quaternion_from_euler(currentRoll, currentPitch,0)
		
        elif(LimitSwitchRight == 1 && LimitSwitchNod ==1):
               	currentRoll = 1.6;
		currentPitch=0;
		msg.pose = quaternion_from_euler(currentRoll, currentPitch,0)
        elif(LimitSwitchLeft ==1):
        elif(LimitSwitchRight ==1):
        else(LimitSwitchNod==1):        
        	
        g_node.get_logger().info('Serial Rx: %s'%str(line))
        publisher.publish(msg)
        rclpy.spin_once(g_node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
