The following is a readme for running the python script and simulator. This is tested in Ubuntu 16.04 LTS.
In order to run use the makefile in a compatible OS.

Open three terminals and run each command in terminals. Remember that the order does matter.

dronekit-sitl rover --home=30.619023,-96.338742,0,0

mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpout:127.0.0.1:14550 --console --map

python final_script.py --connect 127.0.0.1:14550

This home location in the first command can be edited. The location seem above is the H.R.B.B
The first command creates a simulated vehicle. According to all other processes, its as if there is a real vehicle at the home location.

The second command initates the Ground Control Station, MAVProxy. This provides the map and a relay by which to send commands.

The last command starts the controlling python script. It uses the dronekit library to connect to the vehicle and issue commands.

Below are Eric's notes, instructions, complaints, and caveats.

When running for EMILY run (order matters):

mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpout:127.0.0.1:14550 --map

python final_script.py --connect 127.0.0.1:14550

THE CONFIG FILE
===============================
r is the radius of earth in kilometers
goal_radius is the radius at which robot will stop
control_region_radius is the radius at which try robot starts to worry about pose
ballistic_region_gain is the attractive field gain in ballistic region. ???
tangential_select_gain is the tangential field gain in the select region.
tangential_control_gain is the tangential field gain in the control region.
att_select_gain ???
att_control_gain ???
pose_radians = pi/3			# the pose robot should achieve at the goal
select_radians = pi/4			# select region angle
