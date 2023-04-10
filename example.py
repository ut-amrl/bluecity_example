#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys

stream = None

def getFrame():
  while True:
    print("Getting frame...")
    data = stream.get_frame()
    print("Frame: ")
    print (data)

def getOccupancy():
  while True:
    print("Getting occupancy...")
    data = stream.get_occupancy()
    print("Occupancy: ")
    print (data)

def getPhase():
  while True:
    print("Getting phase...")
    data = stream.get_phase()
    print("Phase: ")
    print( data)

if __name__ == '__main__':
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

  frame_thread =threading.Thread(target=getFrame)
  occupancy_thread= threading.Thread(target=getOccupancy)
  phase_thread = threading.Thread(target=getPhase)

  frame_thread.start()
  phase_thread.start()
  occupancy_thread.start()
  frame_thread.join()
  phase_thread.join()
  occupancy_thread.join()