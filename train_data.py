from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib
import cv2
import numpy as np
import os
import pickle
import time

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import VisualizationMsg
from amrl_msgs.msg import ColoredLine2D
from amrl_msgs.msg import Point2D

stream = None
arr = []

def save_data():
  # Save the trajectories as a pickle file appending the current date and time
  print(trajectories)
  # Make sure trajectories is not empty
  if len(trajectories) == 0:
    print("No trajectory data to save")
    return
  with open('data/traj/' + f"trajectory_{time.strftime('%Y%m%d-%H%M%S')}.pkl", "wb") as f:
    pickle.dump(trajectories, f)
  print("Trajectory data saved")

if __name__ == '__main__':
  rospy.init_node('bluecity_trajectories', anonymous=False)
  rospy.on_shutdown(save_data)

  # Trajectory dictionary mapping object id to a nested dictionary
  # of the form {'class': class_type, 'frames': {frame_number: {'x': x_coordinate, 'y': y_coordinate, 'speed': speed}}}
  trajectories = {}

  # Loop while rospy is not shutdown. 
  while rospy.is_shutdown() == False:
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
            # BCTWSConnection.subscriptionOption.LOOP_CHANGE,
            # BCTWSConnection.subscriptionOption.PHASE_CHANGE,
            BCTWSConnection.subscriptionOption.FRAME])
    print("Stream opened")
    frame_counter = 0
    while rospy.is_shutdown() == False:
      # get one frame of data
      data = stream.get_frame()

      # append to trajectory dictionary
      obj_count = 0
      for obj in data.objects:
        obj_count +=1
        if obj.id not in trajectories:
          trajectories[obj.id] = {'class': obj.classType, 'frames': {}}
          # speed is an optional field
        if obj.speed is not None:
          trajectories[obj.id]['frames'][frame_counter] = {'x': obj.centerX, 'y': obj.centerY, 'speed': obj.speed, 'rotation': obj.rotation, 'width': obj.width, 'length': obj.length}
        else:
          trajectories[obj.id]['frames'][frame_counter] = {'x': obj.centerX, 'y': obj.centerY, 'speed': 0, 'rotation': obj.rotation, 'width': obj.width, 'length': obj.length}
      # increment frame counter
      frame_counter += 1
      print(obj_count)
      # if obj_count <= 10:
      #    if trajectories != {}:
      #       save_data()
      #       print(trajectories)
      #       trajectories = {}