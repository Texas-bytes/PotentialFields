# This is a file to initialize the Global Parameters across all scripts.
from os.path import isfile

# TODO 'True' and 'pi/3' cannot be type casted to floats. Fix this error...
def readParametersFromConfig(filename = "config.txt"):
	if isfile(filename):
		print ("Read from ",filename,': ')
		configFile = open(filename, 'r')
		try:
			param = {}
			for paramString in configFile:
				row = paramString.split(' = ')
				param[row[0]] = float(row[1].rstrip('\n'))
				#param[row[0]] = param[row[0]].rstrip(' ')
			print (param)
			return param
		except:
			print ("Config file format is parameter_name = value with one parameter per line.")
	else:
		print (filename," not found!")

def init():
	global PARAMETERS
	PARAMETERS = readParametersFromConfig()
