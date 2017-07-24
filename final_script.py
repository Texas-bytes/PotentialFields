from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np						# needed mostly for plots
import matplotlib.pyplot as plt					# needed mostly fot plots
from math import asin, sin, cos, tan, sqrt, atan2, radians, pi
from defined import *

import global_parameters
# Set up option parsing to get connection string
import argparse
# New Global Parameters. Delete Old parameters object after testing.


# Wrapper for sending throttle commands. Throttle disabled if enableThrottle= False in config file.
def sendThrottleCommand(pwm,throttle):
	if throttle:
		vehicle.channels.overrides = {'3':pwm}
	else:
		print ("Throttle disabled. PWM sent: ",pwm)
		return

# Read Parameters from a config file.
# Eric TODO: Create a json or csv to store global params



def approach_gps(g_lat,g_lon,emily_lat_start, emily_lon_start): #approach a gps position using potential fields
	"""
	# This is the main function through which emily appraoches a gps location by potential fields
	"""
	x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
	x_e_start,y_e_start = latlongtoxy(emily_lat_start,emily_lon_start,g_lat)

	print ("\n HERE I AM\n\n")

	dist =  haver_distance(g_lat, g_lon, emily_lat_start, emily_lon_start)
	initial_dist = dist

	print ('Distance: ',dist)
	heading = get_heading(emily_lat_start, emily_lon_start, g_lat, g_lon)
	print ('After get heading')
	# Eric: I'm not sure if turn_towards is necessary for a successful run.
	#turn_towards(heading)
	print ('After Turn towards')
	#turn towards the goal initially

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

		dx,dy = approach_victim_behaviour(y_goal,x_goal, y_e,x_e)	#get potential field vector
		rc1, rc3 = dxdytorc(dx,dy, e_heading,g_lon)					#get rc parameters
		dist =  haver_distance(g_lat, g_lon, e_lat, e_lon)				#haversine distance

		current_time = time.time() - start_time
		print ("Time, Heading, Distance")
		print (current_time, e_heading*180/pi, dist)
		dstore.append(dist)
		hstore.append(e_heading*180/pi)
		#code for sending the writing the rc commands
		# 3 is the thrust controlacef
		#vehicle.channels.overrides = {'3':rc3}
		sendThrottleCommand(rc3, enableThrottle)
		time.sleep(0.5)
		vehicle.channels.overrides = {'1':rc1}
		print ("Rudder: ",rc1)
		print ("Throttle: ",rc3)
		saveToLog(e_lat, e_lon,dist,rc1,rc3)
		time.sleep(0.5)
	print(initial_dist)
	print("intial ", emily_lat_start,emily_lon_start)
	print("final ",e_lat,e_lon)
	plt.plot(dstore)
	#plt.title('Distance form home vs time')
	plt.xlabel("Time")
	plt.ylabel('Distance')
	plt.show()
	plt.plot(hstore)
	plt.show()

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
	# XXX : what the heck is this?
	#vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)


#Close vehicle object before exiting script
def savecounter():
	print ("\nClose vehicle object")
	#vehicle.channels.overrides = {'3':1500}
	sendThrottleCommand(1500, enableThrottle)
	#time.sleep(1)
	vehicle.channels.overrides = {'1':1500}
	time.sleep(1)
	vehicle.channels.overrides = {}
	time.sleep(1)
	#vehicle.mode = VehicleMode("RTL")
	vehicle.close()


if __name__ == "__main__":
	#Reads parameters from config file and intializes global variable.
	global_parameters.init()
	print ('-------------------------')
	print (global_parameters.PARAMETERS)
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

	#vehicle.channels.overrides = {'1':1100}
	#time.sleep(5)
	#vehicle.channels.overrides = {'1':1900}

	# ---------------------call the approach_gps2 function (this is the main function)---------------------------------
	tempGoalX = 30.618230
	tempGoalY = -96.336466
	approach_gps(tempGoalX,tempGoalY, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)

	time.sleep(1)
	# Shut down simulator if it was started.
	if sitl is not None:
		sitl.stop()

	print("Completed")
	import atexit
	atexit.register(savecounter)
'''
if __name__=="__main__":
	readParametersFromConfig()
'''
'''
TODO:

Create unit testing for new function. Especially conversion and mathy ones.
'''
