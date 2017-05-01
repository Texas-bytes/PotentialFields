from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import numpy as np
import matplotlib.pyplot as plt
from math import asin, sin, cos, tan, sqrt, atan2, radians, pi
from defined import *

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
#print ('Connecting to vehicle on: %s') % connection_string
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming Throttle")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
	


print (vehicle.location.global_frame.lat)
print (vehicle.location.global_frame.lon)
print (vehicle.heading)
head = vehicle.heading
arm_and_takeoff(100)

time.sleep(1)
vehicle.mode = VehicleMode("MANUAL")
time.sleep(1)
vehicle.armed = True
time.sleep(1)

vehicle.channels.overrides = {'3':1500}
vehicle.channels.overrides = {'1':1500}


vehicle.channels.overrides = {'3':1500}
time.sleep(1)
vehicle.channels.overrides = {'1':1500}
time.sleep(1)
print ("\nI am here!")
#Close vehicle object before exiting script 

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

def savecounter():
	print (vehicle.heading)
	print ("\nClose vehicle object")
	vehicle.close()

import atexit
atexit.register(savecounter)

print("Completed")


