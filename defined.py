import numpy as np
import matplotlib.pyplot as plt
from os.path import isfile
from math import asin, sin, cos, tan, sqrt, atan, atan2, radians, pi, degrees

# TODO move to global variable
r = 6371000					        # radius of earth in meters
PID = 20.0
rudder_min = 1100
rudder_max = 1900
throttle_min = 1520
throttle_max = 1900

def saveToLog(emilyXLocation, emilyYLocation,distance,rudderPWM,throttlePWM):
	if isfile('log.csv'):
		f = open('log.csv', 'a')
		f.write(str(emilyXLocation) + ',' + str(emilyYLocation) + ',' + str(distance) + ',' + str(rudderPWM) + ',' + str(throttlePWM) + '\n')
	else:
		f = open("log.csv" , 'w+')
		f.write('emilyXLocation, emilyYLocation, distance, rudderPWM, throttlePWM,\n')
		f.write(str(emilyXLocation) + ',' + str(emilyYLocation) + ',' + str(distance) + ',' + str(rudderPWM) + ',' + str(throttlePWM) + '\n')
	f.close()

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


def dxdytorc(dx,dy,e_orientation_rad,goal_lon):		#convert potential fields vector to throttle and rudder input
	print "In dxdytorc"
	#This is the physical limit of the rudder servos.
	# XXX : Not sure if this is affecting the rudder turning ability???
	rudder_span_rad = pi*2			#span of angle in radians
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max
	#rc1 = 0

	# XXX :
	rc1_unit = (rudder_max - rudder_min)/rudder_span_rad	#This

	theta = atan2(dy,dx)
	if(theta < 0):
		#Make theta positive e.g. -90 -> 270.
		theta = theta + 2*pi
		print "Theta: ", theta
	headingDifference = theta - e_orientation_rad			# Difference between emily heading and field vector
	# it might need to change if heading is with respect to magnetic north
	if(abs(headingDifference) <= 180):
		# we found sin(a) to be the right function for control
		# XXX
		kp = PID / 1000.0
		rc1 = 1500 + (kp * (headingDifference))*400
		print "Rudder: ",rc1
		# add to 1500 rudder command
	# XXX : Not sure if this needs to be here or unindented
	print "End dxdytorc"
	return round(rc1), round(rc3)

# ALL UNITS ARE IN RADIANS. XXX Non working implementation
def vectorToRC(dx,dy,e_orientation_rad,goal_lon):		#convert potential fields vector to throttle and rudder input
	print "In vectorToRC"
	# XXX : Not sure if this is affecting the rudder turning ability???
	rudder_span_rad = pi*2			#span of angle in radians
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max

	#theta is the Target vector heading
	theta = atan2(dy,dx)
	if(theta < 0):
		#Make theta positive e.g. -90 -> 270.
		theta = theta + 2*pi
	print "Theta: ", theta
	headingDifference = theta - e_orientation_rad			# Difference between emily heading and field vector
	print 'Heading Difference: ',headingDifference
	# it might need to change if heading is with respect to magnetic north
	kp = PID / 1000.0

	if(abs(headingDifference) < pi):
		if(headingDifference < pi/6.0):
			rc1 = kp * headingDifference
		else:
			if theta > e_orientation_rad:
				rc1 = rudder_max
			else:
				rc1 = rudder_min
	elif (abs(headingDifference) >= pi):
		if theta > e_orientation_rad:
			# not the same as headingDifference
			angle_difference = headingDifference - 2.0*pi
		else:
			angle_difference = headingDifference + 2.0*pi

		if (abs(angle_difference) <pi/6.0):
			rc1 = kp * angle_difference
		else:
			if angle_difference > 0:
				rc1 = rudder_max
			else:
				rc1 = rudder_min

		# we found sin(a) to be the right function for control
		# XXX
		#rc1 = 1500 + (kp * (headingDifference))*400
		print "Rudder: ",rc1
		# add to 1500 rudder command

	print "End vectorToRC"
	return round(rc1), round(rc3)

#See readme file for in depth explanation.
def vectorToCommands(dx, dy, emilyOrientation ):
	#converting vector x and y components to Throttle commands.
	# FIXME throttle is wonky and always 1900+ because of vector components dx, and dy.
	rc3 = throttle_min + (throttle_max - throttle_min)*sqrt(dx**2 + dy**2)    #rc3 input for throttle_max
	print 'emily heading: ',degrees(emilyOrientation)
	#section for converting x and y components to a rudder command
	targetHeading = atan2(dy,dx)
	# XXX There is some framing issue here. if target is in upper left relative to emily, a negative angle is return. ex: -60 is returned by its positive complement is desired so add 360.
	if targetHeading<0:
		targetHeading = targetHeading + 2*pi
	# this is used as the denominator in a percentage, (target - emily)/maxTurningAngle, to determine what percentage of the turning span to turn.
	# so percentage*turningSpan + pwm_of_zero -> percentage*(max - min)+1500
	maxTurningAngle = pi/2.0
	# The difference between emily's heading and the target's.
	headingDiff = targetHeading - emilyOrientation

	print 'target heading: ',degrees(targetHeading)
	print 'headingDiff: ',degrees(headingDiff)
	# the headingDiff is centered on emilys heading so emilys heading is 0 and anything between emilys heading and true north will be negative
	if headingDiff < 0:
		headingDiff = headingDiff + 2*pi
	if pi/2.0 <= headingDiff <= pi:
		#if heading difference is between 90-180 degrees, take a HARD RIGHT.
		rc1 = rudder_max
	elif  pi < headingDiff <= 3*pi/2.0:
		# if heading between 180-270, take a HARD LEFT.
		rc1 = rudder_min
	elif headingDiff > 3*pi/2.0:
		# slight left proportional to the percentage of the maximum turning radius.
		percentage = (targetHeading - emilyOrientation)/maxTurningAngle
		print 'percentage',percentage
		# 400 is the difference in PWM for the neutral rudder position (i.e. when emily is going straight: PWM 1500), and a max right or left (1100 of 1900)
		rc1 = 400*percentage + 1500
	elif headingDiff < pi/2.0:
		# headingDiff < 90 so slight turn right. Turn as a percentage of maximum possible angle.
		percentage = (targetHeading - emilyOrientation)/maxTurningAngle
		print 'percentage',percentage
		rc1 = 400*percentage + 1500

	return round(rc1),round(rc3)

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

def attractiveField(xGoal, yGoal, xEmily, yEmily, gain = 1.0):
	dx, dy = 0, 0
	dist = sqrt((xGoal - xEmily)**2 + (yGoal - yEmily)**2)
	theta = atan2((yGoal - yEmily),(xGoal - xEmily))
	print 'Theta: ',theta
	# TODO Tweak the gain
	dx = gain*cos(theta)
	dy = gain*sin(theta)

	return dx, dy

# FIXME Im not sure if the dx and dy return the correct signs. Theta appears to be correct though.b
def tangentialField(xGoal, yGoal, xEmily, yEmily,pose = pi/3.0, gain = 1.0):
	######## Pose line calculations ########
	# opposite side of triangle formed from the pose anlge to x-intercept. TODO doc
	o = yGoal*tan(pi/3)
	# XXX I'm not sure if this will work in other hemispheres because the sign of longitude will be flipped.
	poseSlope = yGoal/(o)
	# y intercept of the pose line
	b_pose = yGoal - poseSlope*xGoal
	######## Line perpendicular to Pose Line ########
	# slope of line perpendicular to pose line.
	perpSlope  = - 1/poseSlope
	# 90 - pose gives the angle of the triangle to calculate the perpendicular y intercept
	alpha = pi/2 - pose
	#find adjacent side of triangle.
	a = xGoal*(1/tan(alpha))
	b_perp = yGoal + a
	#################################################
	slope = (yGoal - yEmily)/(xGoal - xEmily)
	tanSlope = -(xGoal - xEmily)/(yGoal - yEmily)

	theta = atan2((yGoal - yEmily),(xGoal - xEmily))
	# if above perpendicular turn towards top pose line if below turn towards bottom line.
	y_perp = perpSlope*xEmily + b_perp
	y_pose = poseSlope*xEmily + b_pose
	# FIXME Start here!!! The scaling with the vectors is off so the attractive field has no effect
	x_dist = xEmily
	y_dist = yEmily
	if yEmily >= y_perp and yEmily>= y_pose:
		#anti-clockwise motion around goal
		dx = cos(theta + pi/2)
		dy = sin(theta + pi/2)
	elif yEmily >= y_perp and yEmily < y_pose:
		#clockwise motion around goal
		dx = cos(theta - pi/2)
		dy = sin(theta - pi/2)
	elif yEmily < y_perp and yEmily >= y_pose:
		#clockwise motion around goal
		dx = cos(theta - pi/2)
		dy = sin(theta - pi/2)
	elif yEmily < y_perp and yEmily < y_pose:
		#anti-clockwise motion around goal
		dx = cos(theta + pi/2)
		dy = sin(theta + pi/2)
	print 'Theta: ',degrees(theta)
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

def approachVictim(xGoal, yGoal, xEmily, yEmily, pose):
	#'''
	attr_dx,attr_dy = attractiveField(xGoal, yGoal, xEmily,yEmily)
	print 'Attractive Field xComponent: ', attr_dx
	print 'Attractive Field yComponent: ', attr_dy
	#'''
	tan_dx, tan_dy = tangentialField(xGoal, yGoal, xEmily, yEmily)
	print 'Tangential Field xComponent: ', tan_dx
	print 'Tangential Field yComponent: ', tan_dy
	#'''
	dx, dy = attr_dx+tan_dx , attr_dy+tan_dy
	#dx, dy = tan_dx , tan_dy
	#dx, dy = attr_dx, attr_dy

	print 'Resultant Field xComponent: ', dx
	print 'Resultant Field yComponent: ', dy
	return dx,dy

def approach_victim_behaviour(x_g, y_g, x_e, y_e, pose_rad, Parameters):			#approach victim behaviour
	"""
	# This function calls different fields in different regions and adds thier contribution
	"""
	dx,dy, dx_tan, dy_tan, dx_attc, dy_attc, dx_attb, dy_attb  = 0,0,0,0,0,0,0,0

	#call attractive field in ballistic region
	#dx_attb,dy_attb  = att_potential_field(x_g,y_g,x_e,y_e,Parameters[1],float("inf"), Parameters[3],'constant')
	dx_attb,dy_attb  = attractiveField(x_g,y_g,x_e,y_e)

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

def plot_fields(xGoal, yGoal,xEmily, yEmily, pose, NX = 30, NY = 30):
	#poseLineSlope = tan(pose)

	figScaling = max(abs(xGoal-xEmily), abs(yGoal-yEmily))*0.5
	minX = min(xGoal,xEmily)-figScaling
	maxX = max(xGoal,xEmily)+figScaling
	minY = min(yGoal,yEmily)-figScaling
	maxY = max(yGoal,yEmily)+figScaling

	# Plot pose line, perpendicular pose line, and control region.
	######## Pose line calculations ########
	o = yGoal*tan(pi/3)
	# XXX I'm not sure if this will work in other hemispheres because the sign of longitude will be flipped.
	poseSlope = yGoal/(o)
	# y intercept of the pose line
	b_pose = yGoal - poseSlope*xGoal
	######## Line perpendicular to Pose Line ########
	# slope of line perpendicular to pose line.
	perpSlope  = - 1/poseSlope
	# 90 - pose gives the angle of the triangle to calculate the perpendicular y intercept
	alpha = pi/2 - pose
	#find adjacent side of triangle.
	a = xGoal*(1/tan(alpha))
	b_perp = yGoal + a

	x = np.linspace(minX, maxX,NX)
	y = np.linspace(minY, maxY,NY)
	#print x
	#print y

	X, Y = np.meshgrid(x,y)
	#print X
	#print Y
	U, V = [], []
	#C1 = []
	#C2 = []
	for i in range(NX):
		for j in range(NY):
			u , v = approachVictim(xGoal, yGoal, X[i,j], Y[i,j], pose)
			U.append(u)
			V.append(v)
		#c1 = yGoal -  poseLineSlope*X[i,j] 							# intercept of pose line
		#c2 = yGoal + (1/poseLineSlope)*X[i,j]
		#C1.append(c1)
		#C2.append(c2)

	plt.figure()
	plt.title("Potential Vector Field")
	#plot has to be flipped so x <=> y.
	Q = plt.quiver(Y, X, V, U, units='width')
	plt.plot(poseSlope*x+b_pose,x,'r-')
	plt.plot(perpSlope*x+b_perp,x,'r-')
	plt.plot(yGoal,xGoal,'go',yEmily,xEmily,'ro')
	#C1 = np.array(C1)
	#print poseLineSlope
	#print len(x)
	#print len(C1)
	#plt.plot(C1,x,'b-')

	qk = plt.quiverkey(Q, 0.9, 0.9, 2, r'$2 \frac{m}{s}$', labelpos='E',
	                   coordinates='figure')


	plt.show()


if __name__ == "__main__":
	# Testing functions.
	tempGoalX = 30.615498
	tempGoalY = -96.336171
	xEmily = 30.619023
	yEmily = -96.338742

	'''
	dx,dy = attractiveField(3,5,0,0)
	plot_fields(tempGoalX, tempGoalY, xEmily, yEmily, pi/3.0)
	print 'From Emily:'
	tangentialField(tempGoalX,tempGoalY,xEmily,yEmily)
	'''
	plot_fields(tempGoalX, tempGoalY, xEmily, yEmily, pi/3.0)
	#tan_potential_field(tempGoalX, tempGoalY, xEmily, yEmily, 0, float('inf'), 1.0, 'constant', pi/3)
