# Makefile for installing the dronkit simulator with MavProxy as the Ground Control Station (GCS)
# This is tested using Ubuntu 16.04.2 LTS downloaded at https://www.ubuntu.com/download/desktop
# Helpful Links: 
# http://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html
# http://diydrones.com/profiles/blogs/idiot-s-guide-to-dronekit-python-a-journey-to-whoz-chillin


# Author: Timothy Soto
# Date: 06 MAY 2017
 

all: python mavproxy dronekit dronekit-sitl git

python:
	sudo apt-get install python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml

mavproxy:
	pip install MAVProxy

dronekit:
	pip install dronekit

dronekit-sitl:
	pip install dronekit-sitl

git:	
	sudo apt-get install git

.PHONY: all
