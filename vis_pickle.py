#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib, rosbag  # MODIFIED: added rosbag
from datetime import datetime  # Import datetime to generate timestamps
import pickle

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import VisualizationMsg
from amrl_msgs.msg import ColoredLine2D
from amrl_msgs.msg import Point2D

stream = None
bag = None  # MODIFIED: Initialize bag variable
density_threshold = 10  # MODIFIED: Threshold for crowd density

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
   # Load data from pickle file
  with open("data_common_frames.pkl", "rb") as f:
    data = pickle.load(f)

    msg = VisualizationMsg()
    msg.header.frame_id = "map"
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.ns = "bluecity_example"

    sensorLoc = Point2D()
    sensorLoc.x = 86
    sensorLoc.y = -120
    sensorAngle = math.radians(5)
    pub = rospy.Publisher('visualization', VisualizationMsg, queue_size=10)
    # bag = rosbag.Bag(f'crowd_density.bag', 'w')


    # Iterate through each agent's frames in the loaded pickle data
    # for agent_id, agent_data in data.items():
    while rospy.is_shutdown() == False:

        for frame_number, frame_data in enumerate(data['742764702']['frames'].items()):
            ResetVisualizationMsg(msg)
            # Example to process a single frame data
            # Assume each frame corresponds to a single object in this example
            x = frame_data[1]['x']
            y = frame_data[1]['y']
            speed = frame_data[1]['speed']  # Not directly used for visualization, but potentially useful for extensions

            # Example processing logic; you might need to adjust based on your full data structure
            # Here, every object is treated similarly without type differentiation as an example
            color = 0xFF0000  # Example color, adjust based on actual data or requirements
            msg.lines.extend(DrawBox(x+sensorLoc.x, -y+sensorLoc.y, 1, 1, 0, color))  # Assuming fixed size and no rotation for simplicity

            pub.publish(msg)
            # ROS bag recording logic remains similar, adapt as necessary
        # bag.write('visualization', msg, rospy.Time.now())

