#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken

import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
#from waterlinked_a50_ros_driver.msg import DVL
#from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
import time
import sys
import argparse

# ---------- Global Variables ---------------
#global variables for control
global int_error #for integral 
global dt 
dt = 0.001 # ask V
int_error = 0

#### control modes
global control_depth
global depth_control_mode
global floatability
floatability = 0
#for depth
control_depth = 0 # the depth at which we want to control
depth_control_mode = 0 #0 for not controlling depth, 1 for P, 2 for PI, 3 for PID

#for yaw
global control_yaw
global yaw_control_mode
global counter
counter = 0
control_yaw = 0 #the yaw at which we want to control
yaw_control_mode = 0 #0 for not controlling depth, 1 for P, 2 for PI, 3 for PID

### trajectory
def trajectory( t ,state_init=[5,5,5,5,5,5], state_final=[5,5,5,5,5,5], t_final=20, enable=[1,1,1,1,1,1]):
    # t = np.array(t)
    # t = t[t<=t_final]

    a2 = 3*(np.array(state_final) - np.array(state_init))/(t_final**2)
    a3 = -2*(np.array(state_final)-np.array(state_init))/(t_final**3)
    
    output = np.array(state_init).reshape(6,1) + np.multiply(np.reshape(a2, (6,1)), t**2) + np.multiply(np.reshape(a3, (6,1)), t**3)
    derivative = np.array(state_init).reshape(6,1) + 2*np.multiply(np.reshape(a2, (6,1)), t) + 3*np.multiply(np.reshape(a3, (6,1)), t**2)
    
    return output, derivative

def trajectory_gen(t_final=20, mission_time=30, state_init = [0,0,0,0,0,0], state_final=[1,2,5,0,4,0], dt=0.001):
    t = np.linspace(0, t_final, int(t_final/dt))
    
    traj, der = trajectory(t, state_init, state_final, t_final)

    t_steady = np.linspace(t_final, mission_time, int((mission_time-t_final)/dt))
    t_steady = t_steady[1:]
    N = t_steady.shape[0]
    aa = np.array(state_final).reshape((6,1))
    state_steady = np.multiply(np.array(state_final).reshape((6,1)), np.ones((6, N)))
    steady_state_der = np.multiply(np.array(state_final).reshape((6,1)), np.zeros((6, N)))

    return np.concatenate((traj, state_steady), axis =1), np.concatenate((der, steady_state_der), axis =1) 

traj, _ = trajectory_gen(t_final=20, mission_time=30, state_init=[0,0,depth_p0,0,0,angle_yaw_a0], state_final=[0,0,control_depth, 0,0,control_yaw])
if yaw_control_mode!=0:
	control_yaw = traj[5,:]
elif depth_control_mode!=0:
	control_depth= traj[2,:]


set_mode = [0]*3
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

#Conditions
init_a0 = True
init_p0 = True
arming = False

angle_wrt_startup = [0]*3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0
depth_wrt_startup = 0
depth_p0 = 0

enable_depth = False 
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

Vmax_mot = 1900
Vmin_mot = 1100

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# ---------- Functions---------------

def P_control(kp=5, state=5, destination =5, floatability=floatability):
    output =kp*(destination-state) + floatability     
    return output 
	
def PI_control(kp=5, ki=5, state=5, destination=5, floatability=floatability):
    global int_error
    global dt
    error = destination- state
    P_control = kp*error
    int_error += error*dt
    output = P_control + int_error*ki + floatability
    return output

	
def PID_control(kp=5, ki=5, kd=5, state=5, destination=5, floatability=floatability):
    global int_error
    global dt
    global prev_error

    error = destination- state
    p_control = kp*error
    int_error += error*dt
    pi_control = p_control + int_error*ki

    diff_error = (error-prev_error)/dt
    output = pi_control + kd*diff_error + floatability
    prev_error = error
    return output

def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global Sum_Errors_Vel
	global Sum_Errors_angle_yaw
	global Sum_Errors_depth

	# Joystick buttons
	btn_arm = data.buttons[7]  # Start button
	btn_disarm = data.buttons[6]  # Back button
	btn_manual_mode = data.buttons[3]  # Y button
	btn_automatic_mode = data.buttons[2]  # X button
	btn_corrected_mode = data.buttons[0]  # A button

	# Disarming when Back button is pressed
	if (btn_disarm == 1 and arming == True):
		arming = False
		armDisarm(arming)

	# Arming when Start button is pressed
	if (btn_arm == 1 and arming == False):
		arming = True
		armDisarm(arming)

	# Switch manual, auto and correction mode
	if (btn_manual_mode and not set_mode[0]):
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False		
		rospy.loginfo("Mode manual")
	if (btn_automatic_mode and not set_mode[1]):
		set_mode[0] = False
		set_mode[1] = True
		set_mode[2] = False		
		rospy.loginfo("Mode automatic")
	if (btn_corrected_mode and not set_mode[2]):
		init_a0 = True
		init_p0 = True
		# set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
		set_mode[0] = False
		set_mode[1] = False
		set_mode[2] = True
		rospy.loginfo("Mode correction")

def armDisarm(armed):
	# This functions sends a long command service with 400 code to arm or disarm motors
	if (armed):
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Arming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except arming")
	else:
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except disarming")	


def velCallback(cmd_vel):
	global set_mode

	# Only continue if manual_mode is enabled
	if (set_mode[1] or set_mode[2]):
		return

	# Extract cmd_vel message
	roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y) #ask V about negative
	pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

def pingerCallback(data):
	global pinger_confidence
	global pinger_distance

	pinger_distance = data.data[0]
	pinger_confidence = data.data[1]

def OdoCallback(data):
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r
    global yaw_control_mode
    global control_yaw
    global counter

    orientation = data.orientation
    angular_velocity = data.angular_velocity

    # extraction of yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angle_roll = euler[0]
    angle_pitch = euler[1]
    angle_yaw = euler[2]

    if (init_a0):
        # at 1st execution, init
        angle_roll_a0 = angle_roll
        angle_pitch_a0 = angle_pitch
        angle_yaw_a0 = angle_yaw
        init_a0 = False

    angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi #ask V
    angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi

    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]

    pub_angle_degre.publish(angle)

    # Extraction of angular velocity
    p = angular_velocity.x
    q = angular_velocity.y
    r = angular_velocity.z
    
    vel = Twist()
    vel.angular.x = p
    vel.angular.y = q
    vel.angular.z = r
    pub_angular_velocity.publish(vel)

    if yaw_control_mode==1:
        r = P_control(kp=5, state=5, destination =control_yaw[counter])
        vel.angular.z = r       #p control for yaw

    elif yaw_control_mode==2:
        r = PI_control(kp=5,ki=5, state=5, destination =control_yaw[counter])       #p control for yaw
        vel.angular.z = r

    

    if yaw_control_mode!=0:
        pub_cmd_vel.publish(vel)              #control for yaw

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return

    # Send PWM commands to motors
    # yaw command to be adapted using sensor feedback	
    Correction_yaw = 1500 
    setOverrideRCIN(1500, 1500, 1500, Correction_yaw, 1500, 1500)


def DvlCallback(data):
	global set_mode
	global u
	global v
	global w

	u = data.velocity.x  # Linear surge velocity
	v = data.velocity.y  # Linear sway velocity
	w = data.velocity.z  # Linear heave velocity

	Vel = Twist()
	Vel.linear.x = u
	Vel.linear.y = v
	Vel.linear.z = w
	pub_linear_velocity.publish(Vel)

def PressureCallback(data):
    global depth_p0
    global depth_wrt_startup
    global init_p0
    rho = 1000.0 # 1025.0 for sea water
    g = 9.80665

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
        return

    pressure = data.fluid_pressure

    if (init_p0):
        # 1st execution, init
        depth_p0 = (pressure - 101300)/(rho*g)
        init_p0 = False

    depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0

    # setup depth servo control here
    # ...
    global control_depth
    global depth_control_mode
    global counter
    if depth_control_mode==1:
        z = P_control(kp=5, state=depth_wrt_startup, destination =control_depth[counter], floatability=floatability)       #p control for yaw
    elif depth_control_mode==2:
        z = PI_control(kp=5,ki=5, state=depth_wrt_startup, destination =control_depth[counter], floatability=floatability)       #p control for yaw
    elif depth_control_mode==3:
        z = PID_control(kp=5, ki=5, kd=5, state=depth_wrt_startup, destination=control_depth[counter], floatability=floatability)

    if depth_control_mode!=0:
        vel = Twist()
        x=0                        #ask V
        y=0
        vel.linear.x = x
        vel.linear.y = y
        vel.linear.z = z
        pub_cmd_vel.publish(vel)


    # update Correction_depth
    Correction_depth = 1500	          #ask V
    # Send PWM commands to motors
    Correction_depth = int(Correction_depth)
    setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)

def mapValueScalSat(value):
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 400 + 1500

	# Saturation
	if pulse_width > 1900:
		pulse_width = 1900
	if pulse_width < 1100:
		pulse_width = 1100

	return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	msg_override = OverrideRCIn()
	msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
	msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
	msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
	msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500

	pub_msg_override.publish(msg_override)


def subscriber():
    global counter
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
    rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
    #rospy.Subscriber("/dvl/data", DVL, DvlCallback)
    rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
    counter+=1
    rospy.spin()

if __name__ == '__main__':
    armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
    pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
    pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
    pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)

    pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)    
    pub_cmd_vel = rospy.Publisher('/br2/cmd_vel',Twist, queue_size = 10, tcp_nodelay = True)

    subscriber()

