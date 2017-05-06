from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import numpy as np						# needed mostly for plots
import matplotlib.pyplot as plt					# needed mostly fot plots
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
vehicle = connect(connection_string, wait_ready=False)

# Define all the parameters here
r = 6371000 				# 6378km optional # Radius of earth in kilometers. Use 3956 for miles
goal_radius = 2				# the radius at which robot will stop
control_region_radius = 200		# the radius at which try robot starts to worry about pose
ballistic_region_gain = 1		# attractive field gain in ballistic region
tangential_select_gain = 0.1		# tangential field gain in select region
tangential_control_gain = 0.8		# tangential field gain in control region
att_select_gain = 1			# tangential field gain in control region
att_control_gain = 0.8			# tangential field gain in control region
pose_radians = pi/4			# the pose robot should achieve at the goal
select_radians = pi/4			# select region angle

Parameters = [goal_radius, control_region_radius, select_radians, ballistic_region_gain, tangential_select_gain, 		tangential_control_gain, att_select_gain, att_control_gain]


def approach_gps(g_lat,g_lon,emily_lat_start, emily_lon_start, pose_rad, Parameters): #approach a gps position using potential fields
	"""
	# This is the main function through which emily appraoches a gps location by potential fields	
	"""
	x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
	x_e_start,y_e_start = latlongtoxy(emily_lat_start,emily_lon_start,g_lat)
	
	print ("\n HERE I AM\n\n")
	
	
	dist =  haver_distance(g_lat, g_lon, emily_lat_start, emily_lon_start)
	print (dist)
	heading = get_heading(emily_lat_start, emily_lon_start, g_lat, g_lon)
	turn_towards(heading)						#turn towards the goal initially
	
	start_time = time.time()
	current_time = 0
	dstore = []
	hstore = []
	while(dist >= goal_radius):

		#------------ code for reading gps location of emily and its orientation ------
		e_lat = vehicle.location.global_frame.lat 
		e_lon = vehicle.location.global_frame.lon
		e_heading = vehicle.heading * pi/180		# convert heading to radians
		#------------------ get e_lat,e_lon, e_orient ---------------------
		
		
		x_e,y_e = latlongtoxy(e_lat,e_lon,g_lat)			#change latitude and longitude to xy
		
		#x,y are given to approach victim function as y,x to algin the north heading and direction in x,y
		
		dx,dy = approach_victim_behaviour(y_goal,x_goal, y_e,x_e, pose_rad, Parameters)	#get potential field vector
		rc1, rc3 = dxdytorc(dx,dy, e_heading,g_lon)					#get rc parameters
		dist =  haver_distance(g_lat, g_lon, e_lat, e_lon)				#haversine distance

		current_time = time.time() - start_time		
		print (current_time, e_heading*180/pi, dist)
		dstore.append(dist)
		hstore.append(e_heading*180/pi)
		#code for sending the writing the rc commands
		vehicle.channels.overrides = {'3':rc3}
		time.sleep(0.5)
		vehicle.channels.overrides = {'1':rc1}
		time.sleep(0.5)
	plt.plot(dstore)
	plt.show()
	plt.plot(hstore)
	plt.show()

def turn_towards(heading):				
	"""
	function to initially point the robot towards goal
	This can also be done by the potential fields but with potential fields robot will move wildly 
	at the starting position
	"""
	head = vehicle.heading
	rc1 = 1900
	while True:
		if(head > heading - 20 and head < heading + 20):
			break
		vehicle.channels.overrides = {'3':1525}
		time.sleep(0.5)
		vehicle.channels.overrides = {'1':rc1}
		time.sleep(0.5)
		head = vehicle.heading

def approach_gps2(g_lat,g_lon,emily_lat_start, emily_lon_start, Parameters):
	"""
	Alternative approach gps function with approximate linear potential fields
	"""
	x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
	x_e_start,y_e_start = latlongtoxy(emily_lat_start,emily_lon_start,g_lat)
	
	print ("\n HERE I AM\n\n")
	#This is the goal pose of EMILY. THis is done by adding 90 to current pose of EMILY
	pose_rad = (vehicle.heading + 90) * (pi/180)
	dist =  haver_distance(g_lat, g_lon, emily_lat_start, emily_lon_start)
	print (dist)
	while(dist >= control_region_radius):
		rc1 = 0				
		head = get_heading(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, g_lat, g_lon)
		diff = head - vehicle.heading 
		diff = 2 * diff
		rc1 = diff + 1500
		#code for sending the writing the rc commands
		vehicle.channels.overrides = {'3':1900}
		time.sleep(1)
		vehicle.channels.overrides = {'1':rc1}
		time.sleep(1)

		dist =  haver_distance(g_lat, g_lon, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
		print (rc1, 1600, dist)
#have a picture of the robot
#figures from simulation\ screenshot
#numbers are always good
#do abstract first
# then approach and implementation
#experimient design
#then rest
#put claims and novelty in abstract / contributions
#image stiching https://uk.pinterest.com/explore/time-lapse-photography/
	while(dist >= goal_radius):
		rc1 = 0	
		rc3 = 200
		rc1 = 10*(400/(control_region_radius))
		dist =  haver_distance(g_lat, g_lon, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
		rc3 = (dist/control_region_radius) * rc3
		rc3 = rc3 + 1500			
		rc1 = rc1 + 1500

		
		#code for sending the writing the rc commands
		vehicle.channels.overrides = {'3':rc3}
		time.sleep(1)
		vehicle.channels.overrides = {'1':rc1}
		time.sleep(1)

		dist =  haver_distance(g_lat, g_lon, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
		print (rc1, rc3, dist)
	
	print ("\n REACHED THE ADMINISTRATION BUILDING!\n\n")
	vehicle.channels.overrides = {'3': 1500}
	time.sleep(1)
	vehicle.channels.overrides = {'1':1500}
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
    #vehicle.mode = VehicleMode("GUIDED")
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
vehicle.mode = VehicleMode("MANUAL")			# change vehicle mode to manual
time.sleep(1)
vehicle.armed = True					# arm the vehicle
time.sleep(1)

# ---------------------call the approach_gps2 function (this is the main function)---------------------------------
#move_random()				# function call for moving to a random location

approach_gps(30.618851,-96.336629, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, pose_radians, Parameters)

#-------------------------------------------------------------------------------------------------------------------

time.sleep(1)

#Close vehicle object before exiting script 
def savecounter():
	
	print ("\nClose vehicle object")
	vehicle.channels.overrides = {'3':1500}
	time.sleep(1)
	vehicle.channels.overrides = {'1':1500}
	time.sleep(1)	
	vehicle.channels.overrides = {}
	time.sleep(1)
	vehicle.mode = VehicleMode("RTL")
	vehicle.close()



# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
import atexit
atexit.register(savecounter)






