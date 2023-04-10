#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib
import pickle

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import VisualizationMsg
from amrl_msgs.msg import ColoredLine2D
from amrl_msgs.msg import Point2D
import itertools

stream = None

def ResetVisualizationMsg(msg):
  msg.header.seq += 1
  msg.header.stamp = rospy.Time.now()
  msg.lines = []

# Return a list of ColorLine2D that represents a box centered at (centerX,
# centerY), with length and width, and rotation angle, and color color
def DrawBox(centerX, centerY, length, width, angle, color):
  # Create a box centered at the point centerX, centerY, rotation angle,
  # with width width and length length
  # The box is defined by 4 points p1, p2, p3, p4 in counter-clockwise order.
  p1 = Point2D()
  p1.x = centerX + length/2 * math.cos(angle) - width/2 * math.sin(angle)
  p1.y = centerY + length/2 * math.sin(angle) + width/2 * math.cos(angle)
  p2 = Point2D()
  p2.x = centerX + length/2 * math.cos(angle) + width/2 * math.sin(angle)
  p2.y = centerY + length/2 * math.sin(angle) - width/2 * math.cos(angle)
  p3 = Point2D()
  p3.x = centerX - length/2 * math.cos(angle) + width/2 * math.sin(angle)
  p3.y = centerY - length/2 * math.sin(angle) - width/2 * math.cos(angle)
  p4 = Point2D()
  p4.x = centerX - length/2 * math.cos(angle) - width/2 * math.sin(angle)
  p4.y = centerY - length/2 * math.sin(angle) + width/2 * math.cos(angle)
  line1 = ColoredLine2D()
  line1.p0 = p1
  line1.p1 = p2
  line1.color = color
  line2 = ColoredLine2D()
  line2.p0 = p2
  line2.p1 = p3
  line2.color = color
  line3 = ColoredLine2D()
  line3.p0 = p3
  line3.p1 = p4
  line3.color = color
  line4 = ColoredLine2D()
  line4.p0 = p4
  line4.p1 = p1
  line4.color = color
  return [line1, line2, line3, line4]

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
  sensorLoc.x = 86
  sensorLoc.y = -120
  sensorAngle = math.radians(5)
  # Create a visualization publisher
  pub = rospy.Publisher('visualization', VisualizationMsg, queue_size=10)


  while rospy.is_shutdown() == False:
    # print("Getting frame...")
    data = stream.get_frame()
    # print("Frame: ")
    # print(len(data.objects))
    ResetVisualizationMsg(msg)
    # Draw a box and a cross at the sensor location.
    msg.lines.extend(DrawBox(sensorLoc.x, sensorLoc.y, 0.5, 0.5, 0, 0x000000))
    msg.lines.append(ColoredLine2D(
        Point2D(sensorLoc.x - 0.5, sensorLoc.y),
        Point2D(sensorLoc.x + 0.5, sensorLoc.y),
        0x000000))
    msg.lines.append(ColoredLine2D(
        Point2D(sensorLoc.x, sensorLoc.y - 0.5),
        Point2D(sensorLoc.x, sensorLoc.y + 0.5),
        0x000000))

    for obj in data.objects:

        try:
            with open('trajectories.pickle', 'rb') as handle:
                objects_dict = pickle.load(handle)
        except FileNotFoundError:
            # If the file doesn't exist, create an empty dictionary
            objects_dict = {}
        # Get the object ID
        obj_id = obj.id
        
        # Check if the ID already exists in the dictionary
        if obj_id in objects_dict:
            # If it does, append the object data to the existing list
            objects_dict[obj_id].append({
                'centerX': obj.centerX,
                'centerY': obj.centerY,
                'width': obj.width,
                'length': obj.length,
                'rotation': obj.rotation,
                'classType': obj.classType,
                'speed': obj.speed,
                'height': obj.height,
                'accuracy': obj.accuracy
            })
        else:
            # If it doesn't, create a new list with the object data and add it to the dictionary
            objects_dict[obj_id] = [{
                'centerX': obj.centerX,
                'centerY': obj.centerY,
                'width': obj.width,
                'length': obj.length,
                'rotation': obj.rotation,
                'classType': obj.classType,
                'speed': obj.speed,
                'height': obj.height,
                'accuracy': obj.accuracy
            }]

        # Save the updated dictionary back to the pickle file
        with open('trajectories.pickle', 'wb') as handle:
            pickle.dump(objects_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
            # Print the result
            # print(result)
        print(obj)
        obj.rotation = obj.rotation + sensorAngle
        obj.centerX = obj.centerX + sensorLoc.x
        obj.centerY = -obj.centerY + sensorLoc.y

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
        #   msg.lines.extend(DrawBox(px, py, obj.length, obj.width, obj.rotation, color))
        msg.lines.extend(DrawBox(obj.centerX, obj.centerY, obj.length, obj.width, obj.rotation, color))
    pub.publish(msg)