import numpy as np
import matplotlib.pyplot as plt
from math import asin, sin, cos, tan, sqrt, atan, atan2, radians, pi, degrees

r = 6371000					        # radius of earth in meters		


def haver_distance(lat1, lon1, lat2, lon2):		# haversine distance formula
    #Calculate the great circle distance between two points 
    #on the earth (specified in decimal degrees)
    
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    return c * r


def dxdytorc(dx,dy,e_orentation_rad,goal_lon):		#convert potential fields vector to throttle and rudder input
	throttle_min = 1520
	throttle_max = 1900
	rudder_min = 1100
	rudder_max = 1900
	rudder_span_rad = 2*pi				#span of angle in radians
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max
	
	rc1_unit = (rudder_max - rudder_min)/rudder_span_rad   
	

	theta = atan2(dy,dx)
	if(theta < 0):
		theta = theta + 2*pi
	a = theta - e_orentation_rad			# Difference between emily heading and field vector
							# it might need to change if heading is with respect to magnetic north
	
	if(abs(a) <= rudder_span_rad):			# if emily orientation is different from vector
		rc1 = sin(a)*rc1_unit			# we found sin(a) to be the right function for control 
		rc1 = 1500 + rc1			# add to 1500 rudder command
	
	return round(rc1), round(rc3)

def latlongtoxy(lat,lon,goal_lat):					#conversion from latitude, longitude to x,y coordinates
	lat, lon,goal_lat = map(radians, [lat, lon, goal_lat])
	x = r*lon*cos(goal_lat)						#x, y based on goal latitude
	y = r*lat
	return [x, y]



def get_heading(lat1, long1, lat2, long2):
	bearing = atan2(sin(long2-long1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360
	bearing = bearing
	return bearing


	
def att_potential_field(x_g, y_g, x_e, y_e, min_r, max_r, strength, type):		#generate attractive field
	
	dx,dy = 0,0

	distance = sqrt((x_g - x_e)*(x_g - x_e) + (y_g - y_e)*(y_g - y_e)) 		# distance b/w emily and goal
	theta = atan2((y_g - y_e),(x_g - x_e))						# angle
	if(type == 'linear'):			
		Vmag = strength*((distance - min_r)/(max_r - min_r))			#linear magnitude profile
	if(type == 'constant'):
		Vmag = strength								#constant magnitude profile
	if(type == 'exponential'):
		Vmag = strength*((distance - min_r)/(max_r - min_r))**2			#exponential magnitude profile
	
	if(distance < min_r):								#zero field inside min radius
		dx = 0
		dy = 0
	if((distance >= min_r) and (distance <= max_r)):				# attractive field
		dx = Vmag*cos(theta)
		dy = Vmag*sin(theta)
	if(distance > max_r):								#zero field outside max radius
		dx = 0
		dy = 0
	return dx,dy
	
def tan_potential_field(x_g, y_g, x_e, y_e, min_r, max_r, strength, type, pose_rad):	#generate tangential field
	
	
	dx,dy = 0,0
	
	m = tan(pose_rad)							# slope of pose line
	
	c1 = y_g -  m*x_g 							# intercept of pose line
	c2 = y_g + (1/m)*x_g							# slope of line perpendicular to pose line
	
	distance = sqrt((x_g - x_e)*(x_g - x_e) + (y_g - y_e)*(y_g - y_e))	# distance of emily from goal
	theta = atan2((y_g - y_e),(x_g - x_e))					# angle
	
	
	if(type == 'linear'):							# magnitude profile
		Vmag = strength*((distance - min_r)/(max_r - min_r))
	if(type == 'constant'):
		Vmag = strength
	if(type == 'exponential'):
		Vmag = strength*((distance - min_r)/(max_r - min_r))**2
	
	if(distance < min_r):	
		dx = 0
		dy = 0
	if((distance >= min_r) and (distance <= max_r)):		#tangential field nonzero only in radius range (min_r,max_r)
		if(m>=0):
			#tangential field has 4 different direction in 4 quadrants and the quadrants are defined according to 				direction of pose line and its perpendicular line
			if((y_e - m*x_e - c1 >= 0) and (y_e + (1/m)*x_e - c2 >= 0)):  
				dx = Vmag*cos(theta + pi/2)
				dy = Vmag*sin(theta + pi/2)
			if((y_e - m*x_e - c1 >= 0) and (y_e + (1/m)*x_e - c2 < 0)):
				dx = Vmag*cos(theta - pi/2)
				dy = Vmag*sin(theta - pi/2)
			if((y_e - m*x_e - c1 < 0) and (y_e + (1/m)*x_e - c2 < 0)):
				dx = Vmag*cos(theta + pi/2)
				dy = Vmag*sin(theta + pi/2)
			if((y_e - m*x_e - c1 < 0) and (y_e + (1/m)*x_e - c2 >= 0)):
				dx = Vmag*cos(theta - pi/2)
				dy = Vmag*sin(theta - pi/2)
		else:
			if((y_e - m*x_e - c1 >= 0) and (y_e + (1/m)*x_e - c2 >= 0)):
				dx = Vmag*cos(theta - pi/2)
				dy = Vmag*sin(theta - pi/2)
			if((y_e - m*x_e - c1 >= 0) and (y_e + (1/m)*x_e - c2 < 0)):
				dx = Vmag*cos(theta + pi/2)
				dy = Vmag*sin(theta + pi/2)
			if((y_e - m*x_e - c1 < 0) and (y_e + (1/m)*x_e - c2 < 0)):
				dx = Vmag*cos(theta - pi/2)
				dy = Vmag*sin(theta - pi/2)
			if((y_e - m*x_e - c1 < 0) and (y_e + (1/m)*x_e - c2 >= 0)):
				dx = Vmag*cos(theta + pi/2)
				dy = Vmag*sin(theta + pi/2)
	if(distance > max_r):
		dx = 0
		dy = 0
	return dx,dy

def approach_victim_behaviour(x_g, y_g, x_e, y_e, pose_rad, Parameters):			#approach victim behaviour
	"""
	# This function calls different fields in different regions and adds thier contribution
	"""
	dx,dy, dx_tan, dy_tan, dx_attc, dy_attc, dx_attb, dy_attb  = 0,0,0,0,0,0,0,0

	#call attractive field in ballistic region
	dx_attb,dy_attb  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[1],float("inf"), Parameters[3],'constant')	

	m = tan(pose_rad)
	c1 = y_g - tan(pose_rad)*x_g 								# intercept of pose line
	m2 = tan(pose_rad + Parameters[2]/2)							#slope of select line 1
	c2 = y_g - m2*x_g											
	m3 = tan(pose_rad - Parameters[2]/2)							#slope of select line 2
	c3 = y_g - m3*x_g	
	if(m>=0):
		if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 >= 0):		#region outside select region 
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[5],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[7],'linear')
		
		if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 < 0):		#inside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[4],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[6],'linear')
		
		if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 >= 0):		#inside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[4],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[6],'linear')
		
		if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 < 0):		#outside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[5],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[7],'linear')
	else:
		if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 >= 0):		#region outside select region 
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[4],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[6],'linear')
		
		if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 < 0):		#inside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[5],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[7],'linear')
		
		if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 >= 0):		#inside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[5],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[5],'linear')
		
		if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 < 0):		#outside select region
			dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[4],'linear', pose_rad)
			dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[0], Parameters[1], Parameters[6],'linear')
	
	dx = dx_attb + dx_attc + dx_tan						#add contribution from all the fields
	dy = dy_attb + dy_attc + dy_tan
	return dx,dy


def plot_fields(NX, NY, xmax, ymax, x_e_start, y_e_start, x_goal, y_goal, pose_rad, Parameters):
	"""
	#function only needed for simulation in python
	"""
	xmin = -xmax								# range of x
	ymin = -ymax								# range of y

	# Make grid and calculate vector components
	x = np.linspace(xmin, xmax, NX)						# NX points to create in X
	y = np.linspace(ymin, ymax, NY)						# NY points to create in Y
	X, Y = np.meshgrid(x, y)							# create X,Y meshgrid
	
	dx = np.zeros(shape = (NX,NY))						# 
	dy = np.zeros(shape = (NX,NY))
	for i in range(NX):
		for j in range(NY):
			dx[i,j], dy[i,j] = approach_victim_behaviour(x_goal,y_goal,X[i,j],Y[i,j], pose_rad, Parameters)
	fig, ax = plt.subplots()
	ax.quiver(x, y, dx, dy, units = 'width')
	ax.set(aspect=1, title='Potential Fields')
	# create pose line plot
	c1 = y_goal -  tan(pose_rad)*x_goal 							#intercept of pose line
							
	y_line  = tan(pose_rad)*x + c1
	
	y_line = (y_line > ymax)*ymax + (y_line <= ymax)*y_line					#make zero if out of range
	y_line = (y_line < -ymax)*(-ymax) + (y_line > -ymax)*y_line				#make zero if out of range
	ax.plot(x,y_line)										#plot of pose line
	
	# plot select lines
	c2 = y_goal -  tan(pose_rad + Parameters[2]/2)*x_goal 			
	c3 = y_goal -  tan(pose_rad - Parameters[2]/2)*x_goal 	
	
	y2_line  = tan(pose_rad + Parameters[2]/2)*x + c2					#equation of line
	
	y2_line = (y2_line > ymax)*ymax + (y2_line <= ymax)*y2_line					#make zero if out of range
	y2_line = (y2_line < -ymax)*(-ymax) + (y2_line > -ymax)*y2_line				#make zero if out of range
	
	y3_line  = tan(pose_rad - Parameters[2]/2)*x + c3					#equation of line
	
	y3_line = (y3_line > ymax)*ymax + (y3_line <= ymax)*y3_line			#make zero if out of range
	y3_line = (y3_line < -ymax)*(-ymax) + (y3_line > -ymax)*y3_line		#make zero if out of range
	
	ax.plot(x,y2_line)										#plot of select region lines
	ax.plot(x,y3_line)										#plot of select region lines
	
	#----------------robot simulation -----------------
	x_e = x_e_start
	y_e = y_e_start
	point_x = np.array(x_e)
	point_y = np.array(y_e)
	dt = 2
	dist = sqrt((x_goal - x_e)*(x_goal - x_e) + (y_goal - y_e)*(y_goal - y_e)) 
	
	while(dist >= Parameters[0] + 0.4):
		dx,dy = approach_victim_behaviour(x_goal,y_goal,x_e,y_e, pose_rad, Parameters)
		V = 4*sqrt(dx**2 + dy**2)
		TH = atan2(dy,dx)
		x_e = x_e + V*cos(TH)*dt
		y_e = y_e + V*sin(TH)*dt
		dist = sqrt((x_goal - x_e)*(x_goal - x_e) + (y_goal - y_e)*(y_goal - y_e)) 
		
		point_x = np.append(point_x,x_e)
		point_y = np.append(point_y,y_e)
		#print x_e, y_e, dist
	print Parameters[0]
	ax.scatter(point_x,point_y)
	plt.show()
