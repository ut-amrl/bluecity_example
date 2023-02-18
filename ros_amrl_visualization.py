#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import VisualizationMsg
from amrl_msgs.msg import ColoredLine2D
from amrl_msgs.msg import Point2D

stream = None

def ResetVisualizationMsg(msg):
  msg.header.seq += 1
  msg.header.stamp = rospy.Time.now()
  msg.lines = []

if __name__ == '__main__':
  rospy.init_node('bluecity_example', anonymous=False)
  # Check to see if a file named ".credentials" exists in the current directory,
  # and if so, use it to log in
  try:
      with open(".credentials") as f:
          username = f.readline().strip()
          password = f.readline().strip()
  except IOError:
      # If the file doesn't exist, use the command line arguments
      # Accept username as the first argument and password as the second argument
      if len(sys.argv) != 3:
          print("Usage: python example.py <username> <password>")
          sys.exit(1)
      username = sys.argv[1]
      password = sys.argv[2]

  print("Opening Blucity stream...")
  stream = BCTWSConnection(
      "BCT_3D_4G_0206001",
      username,
      password,
      singleton=False,
      subscriptions = [
          BCTWSConnection.subscriptionOption.LOOP_CHANGE,
          BCTWSConnection.subscriptionOption.PHASE_CHANGE,
          BCTWSConnection.subscriptionOption.FRAME])
  print("Stream opened")

  msg = VisualizationMsg()
  msg.header.frame_id = "map"
  msg.header.seq = 0
  msg.header.stamp = rospy.Time.now()
  msg.ns = "bluecity_example"

  sensorLoc = Point2D()
  sensorLoc.x = 86.091
  sensorLoc.y = -120
  sensorAngle = 0
  # Create a visualization publisher
  pub = rospy.Publisher('visualization', VisualizationMsg, queue_size=10)
  while rospy.is_shutdown() == False:
    # print("Getting frame...")
    data = stream.get_frame()
    # print("Frame: ")
    # print(len(data.objects))
    ResetVisualizationMsg(msg)
    for obj in data.objects:
      # print(obj)
      obj.rotation = obj.rotation + sensorAngle
      obj.centerX = obj.centerX + sensorLoc.x
      obj.centerY = obj.centerY + sensorLoc.y
      if obj.classType == "10":
          # pedestrian, blue
          color = 0x0000FF
      elif obj.classType == "2":
          # car, red
          color = 0xFF0000
      elif obj.classType == "3":
          # van, purple
          color = 0x800080
      elif obj.classType == "4":
          # truck, orange
          color = 0xFFA500
      elif obj.classType == "5":
          # bus, yellow
          color = 0xFFFF00
      elif obj.classType == "13":
          # bicycle, green
          color = 0x00A000
      else:
          # unknown, black
          print("Unknown class type: " + str(obj.classType))
          color = 0x000000
      # Create a box centered at the point obj.centerX, obj.centerY, rotation
      # obj.rotation, with width obj.width and length obj.length
      # The box is defined by 4 points p1, p2, p3, p4 in counter-clockwise order.
      p1 = Point2D()
      p1.x = obj.centerX + obj.length/2 * math.cos(obj.rotation) - obj.width/2 * math.sin(obj.rotation)
      p1.y = obj.centerY + obj.length/2 * math.sin(obj.rotation) + obj.width/2 * math.cos(obj.rotation)
      p2 = Point2D()
      p2.x = obj.centerX + obj.length/2 * math.cos(obj.rotation) + obj.width/2 * math.sin(obj.rotation)
      p2.y = obj.centerY + obj.length/2 * math.sin(obj.rotation) - obj.width/2 * math.cos(obj.rotation)
      p3 = Point2D()
      p3.x = obj.centerX - obj.length/2 * math.cos(obj.rotation) + obj.width/2 * math.sin(obj.rotation)
      p3.y = obj.centerY - obj.length/2 * math.sin(obj.rotation) - obj.width/2 * math.cos(obj.rotation)
      p4 = Point2D()
      p4.x = obj.centerX - obj.length/2 * math.cos(obj.rotation) - obj.width/2 * math.sin(obj.rotation)
      p4.y = obj.centerY - obj.length/2 * math.sin(obj.rotation) + obj.width/2 * math.cos(obj.rotation)
      line1 = ColoredLine2D()
      line1.p0 = p1
      line1.p1 = p2
      line1.color = color
      msg.lines.append(line1)
      line2 = ColoredLine2D()
      line2.p0 = p2
      line2.p1 = p3
      line2.color = color
      msg.lines.append(line2)
      line3 = ColoredLine2D()
      line3.p0 = p3
      line3.p1 = p4
      line3.color = color
      msg.lines.append(line3)
      line4 = ColoredLine2D()
      line4.p0 = p4
      line4.p1 = p1
      line4.color = color
      msg.lines.append(line4)
    pub.publish(msg)