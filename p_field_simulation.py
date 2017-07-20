import numpy as np
import matplotlib.pyplot as plt
from math import asin, sin, cos, tan, sqrt, atan2, radians, pi
from defined import *

# Define all the parameters here
r = 6371000 						#6378km optional			# Radius of earth in kilometers. Use 3956 for miles
goal_radius = 4
control_region_radius = 40
ballistic_region_gain = 1							#attractive field gain in ballistic region
tangential_select_gain = 0.3								#tangential field gain in select region
tangential_control_gain = 0.8								#tangential field gain in control region
att_select_gain = 1						#tangential field gain in control region
att_control_gain = 0.8								#tangential field gain in control region
pose_radians = pi/2
select_radians = pi/3

Parameters = [goal_radius, control_region_radius, select_radians, ballistic_region_gain, tangential_select_gain, tangential_control_gain, att_select_gain, att_control_gain]


def dxdytorc(dx,dy,e_orentation_rad):		#convert potential fields vector to throttle and rudder input
	throttle_min = 1500
	throttle_max = 1900
	rudder_min = 1000
	rudder_max = 2000
	rudder_span_rad = pi/2								#span of angle in radians
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max

	rc1_unit = (rudder_max - rudder_min)/rudder_span_rad
	theta = atan2(dy/dx)
	if(abs(theta - e_orentation_rad) <= rudder_span_rad/2):			#if emily orientation is different from vector
		rc1 = (theta - e_orentation_rad)*rc1_unit
		rc1 = rc1 + 1500
	else:
		if((theta - e_orentation_rad) > 0):
			rc1 = rudder_max
		else:
			rc1 = rudder_min

	return rc1, rc3


g_lat, g_lon = 52.20472, 052.14056			# goal position
e_lat, e_lon = 52.21477, 052.14077			# emily position

def approach_gps(g_lat,g_lon,emily_lat_start, emily_lon_start, Parameters):		 #approach a gps position using potential fields
	x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
	x_e_start,y_e_start = latlongtoxy(emily_lat_start,emily_lon_start,g_lat)

	pose_rad = pi/2 + atan2((y_g - y_e_start),(x_g - x_e_start))		#perpendicular to initial approach change it input if needed

	while(dist >= goal_radius):
		#------------ code for reading gps location of emily and its orientation ------
		#------------------ get e_lat,e_lon, e_orient ---------------------


		x_e,y_e = latlongtoxy(e_lat,e_lon,g_lat)										#change latitude and longitude to xy
		dx,dy = approach_victim_behaviour(x_goal,y_goal,x_e,y_e, pose_rad, Parameters)	#get potential field vector
		rc1, rc3 = vectorToRC(dx,dy,e_orient)												#get rc parameters
		dist =  haver_distance(g_lat, g_lon, e_lat, e_lon)

		#code for sending the writing the rc commands


#plot_fields(20,20, 50, 50, 0, 0,pose_radians,Parameters)

def test_case(T,NX,NY,xmax,ymax):					#function to run testcases,NX = number of test points on X-axis, xmax = max value of x in plot
	if(T==1):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = 50,50				#goal position
		pose = 25*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==2):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = -50,50				#goal position
		pose = 25*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==3):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = -50,-50				#goal position
		pose = 25*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==4):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = 50,-50				#goal position
		pose = 25*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==5):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = 25,25				#goal position
		pose = 50*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==6):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = -25,-25			#goal position
		pose = 50*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==7):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = 15,15				#goal position
		pose = 75*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==8):
		x_emily, y_emily = 0,0              #emily position
		x_goal, y_goal = 10,10				#goal position
		pose = 75*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==9):
		x_emily, y_emily = -25,-25              #emily position
		x_goal, y_goal = -50,50				#goal position
		pose = 100*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==10):
		x_emily, y_emily = -25,-25              #emily position
		x_goal, y_goal = -50,-50				#goal position
		pose = 100*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==11):
		x_emily, y_emily = 80,80              #emily position
		x_goal, y_goal = 0,0				#goal position
		pose = 45*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)

	if(T==12):
		x_emily, y_emily = 20,-20              #emily position
		x_goal, y_goal = 40,40				#goal position
		pose = 10*pi/180
		plot_fields(NX,NY,xmax,ymax, x_emily, y_emily, x_goal, y_goal,pose,Parameters)


test_case(10,30,30, 80, 80)
