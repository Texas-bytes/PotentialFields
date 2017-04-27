import numpy as np
import matplotlib.pyplot as plt
from math import asin, sin, cos, tan, sqrt, atan2, radians, pi

r = 6371000 						#6378km optional			# Radius of earth in kilometers. Use 3956 for miles

def haver_distance(lat1, lon1, lat2, lon2):							# haversine distance formula
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

def equirect_distance(lat1, lon1, lat2, lon2):					   #approximate equirectangle distance formula

	# convert decimal degrees to radians 
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	dx = (lon2 - lon1) * cos( 0.5*(lat2+lat1) )
	dy = lat2 - lat1
	return r * sqrt( dx*dx + dy*dy )

def latlongtoxy(lat,lon,goal_lat):								#conversion from latitude, longitude to x,y coordinates
	lat, lon,goal_lat = map(radians, [lat, lon, goal_lat])
	x = r*lon*cos(goal_lat)										#x, y based on goal latitude
	y = r*lat
	return [x, y]
	
def att_potential_field(x_g, y_g, x_e, y_e, min_r, max_r, strength, type):		#generate attractive field
	
	dx,dy = 0,0

	distance = sqrt((x_g - x_e)*(x_g - x_e) + (y_g - y_e)*(y_g - y_e)) 		# distance b/w emily and goal
	theta = atan2((y_g - y_e),(x_g - x_e))									# angle
	if(type == 'linear'):			
		Vmag = strength*((distance - min_r)/(max_r - min_r))				#linear magnitude profile
	if(type == 'constant'):
		Vmag = strength														#constant magnitude profile
	if(type == 'exponential'):
		Vmag = strength*((distance - min_r)/(max_r - min_r))**2				#exponential magnitude profile
	
	if(distance < min_r):													#zero field inside min radius
		dx = 0
		dy = 0
	if((distance >= min_r) and (distance <= max_r)):						# attractive field
		dx = Vmag*cos(theta)
		dy = Vmag*sin(theta)
	if(distance > max_r):													#zero field outside max radius
		dx = 0
		dy = 0
	return dx,dy
	
def tan_potential_field(x_g, y_g, x_e, y_e, min_r, max_r, strength, type, pose_rad):	#generate tangential field
	
	
	dx,dy = 0,0
	m = tan(pose_rad)							# slope of pose line
	c1 = y_g -  m*x_g 							# intercept of pose line
	c2 = y_g + (1/m)*x_g						# slope of line perpendicular to pose line
	
	distance = sqrt((x_g - x_e)*(x_g - x_e) + (y_g - y_e)*(y_g - y_e))		#distance of emily from goal
	theta = atan2((y_g - y_e),(x_g - x_e))									#angle
	if(type == 'linear'):													#magnitude profile
		Vmag = strength*((distance - min_r)/(max_r - min_r))
	if(type == 'constant'):
		Vmag = strength
	if(type == 'exponential'):
		Vmag = strength*((distance - min_r)/(max_r - min_r))**2
	
	if(distance < min_r):	
		dx = 0
		dy = 0
	if((distance >= min_r) and (distance <= max_r)):						#tangential field nonzero only in radius range (min_r,max_r)
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
	if(distance > max_r):
		dx = 0
		dy = 0
	return dx,dy
	
def resutant_field(x_g, y_g, x_e, y_e, goal_radius, control_radius, ballistic_strength, tangential_strength, pose_rad, select_rad):
	dx,dy, dx_tan, dy_tan, dx_attc, dy_attc, dx_attb, dy_attb  = 0,0,0,0,0,0,0,0
	
	dx_tan,dy_tan  = tan_potential_field(x_g,y_g,x_e,y_e,goal_radius,control_radius, tangential_strength,'constant', pose_rad)
	dx_attb,dy_attb  = att_potential_field(x_g,y_g,x_e,y_e,control_radius,float("inf"), ballistic_strength,'constant')
	#dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,goal_radius,control_radius, ballistic_strength,'linear')
	
	c1 = y_g - tan(pose_rad)*x_g 								# intercept of pose line
	m2 = tan(pose_rad + select_rad/2)							#slope of select line 
	c2 = y_g - m2*x_g											
	m3 = tan(pose_rad - select_rad/2)
	c3 = y_g - m3*x_g											#intercepts of select region lines
	
	if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 >= 0):		#region outside select region 
		dx_attc,dy_attc = 0,0
		
	if(y_e - m2*x_e - c2 >= 0 and y_e - m3*x_e - c3 < 0):		#inside select region
		dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,goal_radius,control_radius, ballistic_strength,'linear')
		
	if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 >= 0):		#inside select region
		dx_attc,dy_attc  = att_potential_field(x_g,y_g,x_e,y_e,goal_radius,control_radius, ballistic_strength,'linear')
		
	if(y_e - m2*x_e - c2 < 0 and y_e - m3*x_e - c3 < 0):		#outside select region
		dx_attc,dy_attc = 0,0
		
	dx = dx_attb + dx_attc + dx_tan								#add contribution from all the fields
	dy = dy_attb + dy_attc + dy_tan
	return dx,dy

def approach_victim_behaviour(x_g, y_g, x_e, y_e, pose_rad, Parameters):			#approach victim behaviour
	dx,dy, dx_tan, dy_tan, dx_attc, dy_attc, dx_attb, dy_attb  = 0,0,0,0,0,0,0,0
	dx_attb,dy_attb  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[1],float("inf"), Parameters[3],'constant')	#attractive field in ballistic region
	
	c1 = y_g - tan(pose_rad)*x_g 								# intercept of pose line
	m2 = tan(pose_rad + Parameters[2]/2)							#slope of select line 
	c2 = y_g - m2*x_g											
	m3 = tan(pose_rad - Parameters[2]/2)
	c3 = y_g - m3*x_g	
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
	
	dx = dx_attb + dx_attc + dx_tan								#add contribution from all the fields
	dy = dy_attb + dy_attc + dy_tan
	return dx,dy
	
def plot_fields(NX, NY, xmax, ymax, x_goal, y_goal, pose_rad, Parameters):
	
	xmin = -xmax								  		# range of x
	ymin = -ymax										# range of y

	# Make grid and calculate vector components
	x = np.linspace(xmin, xmax, NX)						# NX points to create in X
	y = np.linspace(ymin, ymax, NY)						# NY points to create in Y
	X, Y = np.meshgrid(x, y)							# create X,Y meshgrid
	
	dx = np.zeros(shape = (NX,NY))						#  
	dy = np.zeros(shape = (NX,NY))
	for i in range(NX):
		for j in range(NY):
			#dx[i,j], dy[i,j] = resutant_field(x_goal,y_goal,X[i,j],Y[i,j], 1,control_radius,ballistic_strength,tangential_strength, pose_rad, select_rad) 
			dx[i,j], dy[i,j] = approach_victim_behaviour(x_goal,y_goal,X[i,j],Y[i,j], pose_rad, Parameters)
	fig, ax = plt.subplots()
	ax.quiver(x, y, dx, dy, units = 'width')
	ax.set(aspect=1, title='Potential Fields')
	# create pose line plot
	c1 = y_goal -  tan(pose_rad)*x_goal 							#intercept of pose line
							
	y_line  = tan(pose_rad)*x + c1
	ax.plot(x,y_line)										#plot of pose line
	# plot select lines
	c2 = y_goal -  tan(pose_rad + Parameters[2]/2)*x_goal 			
	c3 = y_goal -  tan(pose_rad - Parameters[2]/2)*x_goal 	
	y2_line  = tan(pose_rad + Parameters[2]/2)*x + c2
	y3_line  = tan(pose_rad - Parameters[2]/2)*x + c3
	#ax.plot(x,y2_line)										#plot of select region lines
	#ax.plot(x,y3_line)										#plot of select region lines
	plt.show()

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

x_goal,y_goal = latlongtoxy(g_lat,g_lon,g_lat)
x_emily,y_emily = latlongtoxy(e_lat,e_lon,g_lat)

goal_radius = 1
control_region_radius = 40
ballistic_region_gain = 1								#attractive field gain in ballistic region
tangential_select_gain = 0.1								#tangential field gain in select region
tangential_control_gain = 0.5								#tangential field gain in control region
att_select_gain = 0.8								#tangential field gain in control region
att_control_gain = 0.2									#tangential field gain in control region
pose_radians = pi/4
select_radians = pi/3

Parameters = [goal_radius, control_region_radius, select_radians, ballistic_region_gain, tangential_select_gain, tangential_control_gain, att_select_gain, att_control_gain]
plot_fields(20,20, 50, 50, 0, 0,pose_radians,Parameters)



	
