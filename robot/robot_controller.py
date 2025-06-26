#!/usr/bin/env python3
# Filename: robot_controller.py
# Author: ARM
# Date: 18 June 2025
# Description:
# 
#

import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from socket import socket, AF_INET, SOCK_DGRAM
import argparse
import json 
import time

DEFAULT_PORT_NO = 9020
GAMEPAD_JSON_PKTSIZE = 140
DEFAULT_LINEAR_SPEED = 0.3 # m/s
DEFAULT_ANGULAR_SPEED = 1 # rad/s


def main():
	
   # get CLI arguments #
   parser = argparse.ArgumentParser()
   parser.add_argument("-p")
   args = parser.parse_args()
   if args.p:
      port = int(args.p)
   else:
      port = DEFAULT_PORT_NO
   
   # init ROS api #
   rclpy.init() 
   node = rclpy.create_node('robot_controller')
   qos = QoSProfile(depth=10)
   pub = node.create_publisher(Twist, 'cmd_vel', qos)
       
   print("5g robot controller - listening on port %d..." % port)
    
   # process gamepad controls via 5G modem # 
   with socket(AF_INET, SOCK_DGRAM) as s:
      s.bind(('', port))
      
      # reset drive
      twist = Twist()
      twist.linear.x = 0.0
      twist.linear.y = 0.0
      twist.linear.z = 0.0
      twist.angular.x = 0.0
      twist.angular.y = 0.0
      twist.angular.z = 0.0
      pub.publish(twist)      

      while True:
         json_pkt = s.recv(GAMEPAD_JSON_PKTSIZE) 
         if not json_pkt:
            break
         y = json.loads(json_pkt)

         #twist = Twist()
         twist.linear.x = DEFAULT_LINEAR_SPEED * int(y["Left_Y"])/32768
         twist.linear.y = 0.0
         twist.linear.z = 0.0
         twist.angular.x = 0.0
         twist.angular.y = 0.0
         twist.angular.z = DEFAULT_ANGULAR_SPEED * int(y["Left_X"])/32768
         #print("Left_X=%d, Left_Y=%d" % (int(y["Left_X"]), int(y["Left_Y"])))
         print("twist.linear.x=%f, twist.angular.z=%f" % (twist.linear.x,twist.angular.z))
         pub.publish(twist)


if __name__ == '__main__':
    main()
