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


# Define all the parameters here
r = 6371000 						#6378km optional			# Radius of earth in kilometers. Use 3956 for miles
goal_radius = 1
control_region_radius = 20
ballistic_region_gain = 1							#attractive field gain in ballistic region
tangential_select_gain = 1								#tangential field gain in select region
tangential_control_gain = 1								#tangential field gain in control region
att_select_gain = 1							#tangential field gain in control region
att_control_gain = 0.3								#tangential field gain in control region
pose_radians = pi/4
select_radians = pi/3

Parameters = [goal_radius, control_region_radius, select_radians, ballistic_region_gain, tangential_select_gain, tangential_control_gain, att_select_gain, att_control_gain]


def dxdytorc(dx,dy,e_orentation_rad,goal_lon):		#convert potential fields vector to throttle and rudder input
	throttle_min = 1500
	throttle_max = 1900
	rudder_min = 1100
	rudder_max = 1900
	rudder_span_rad = 2*pi								#span of angle in radians
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max
	
	rc1_unit = (rudder_max - rudder_min)/rudder_span_rad
	

	theta = atan2(dy,dx) + goal_lon - pi/2
	a = (theta - e_orentation_rad)*180/pi
	a = (a + 180) % 360 - 180
	a = a*pi/180

	if(abs(a) <= rudder_span_rad/2):			#if emily orientation is different from vector
		rc1 = a*rc1_unit
		rc1 = rc1 + 1500
	else:
		if(a > 0):
			rc1 = rudder_max
		else:
			rc1 = rudder_min
	
	return round(rc1), round(rc3)

	
#g_lat, g_lon = 52.20472, 052.14056			# goal position
#e_lat, e_lon = 52.21477, 052.14077			# emily position

def approach_gps(g_lat,g_lon,emily_lat_start, emily_lon_start, Parameters):		 #approach a gps position using potential fields
	x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
	x_e_start,y_e_start = latlongtoxy(emily_lat_start,emily_lon_start,g_lat)
	
	print ("\n HERE I AM\n\n")
	#This is the goal pose of EMILY. THis is done by adding 90 to current pose of EMILY
	pose_rad = pi/2 # + atan2((y_goal - y_e_start),(x_goal - x_e_start))		#perpendicular to initial approach change it input if needed
	dist =  haver_distance(g_lat, g_lon, emily_lat_start, emily_lon_start)
	print (dist)
	while(dist >= goal_radius):
		#------------ code for reading gps location of emily and its orientation ------
		e_lat = vehicle.location.global_frame.lat 
		e_lon = vehicle.location.global_frame.lon
		e_heading = vehicle.heading * pi/180
		#------------------ get e_lat,e_lon, e_orient ---------------------
		
		
		x_e,y_e = latlongtoxy(e_lat,e_lon,g_lat)			#change latitude and longitude to xy
		dx,dy = approach_victim_behaviour(x_goal,y_goal,x_e,y_e, pose_rad, Parameters)	#get potential field vector
		rc1, rc3 = dxdytorc(dx,dy, e_heading,g_lon)						#get rc parameters
		dist =  haver_distance(g_lat, g_lon, e_lat, e_lon)
		print (rc1, rc3,dist)
		
		#code for sending the writing the rc commands
		vehicle.channels.overrides = {'3':rc3}
		time.sleep(1)
		vehicle.channels.overrides = {'1':rc1}
		time.sleep(1)
		
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

#approach_gps(30.619651,-96.338512, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, Parameters)

#vehicle.channels.overrides = {'3':1600}
#time.sleep(1)
#vehicle.channels.overrides = {'1':1900}
#time.sleep(3)
print ("\nI am here!")
while (head < 350 and head > 10):
	vehicle.channels.overrides = {'3':1510}
	vehicle.channels.overrides = {'1':1900}
	head = vehicle.heading
	print (head)

vehicle.channels.overrides = {'3':1500}
time.sleep(1)
vehicle.channels.overrides = {'1':1500}
time.sleep(1)
print ("\nI am here!")
#Close vehicle object before exiting script 
print (vehicle.heading)
print ("\nClose vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")







