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

#----------global variables for control----------
global int_error #for integral 
global dt 
global prev_error

#check the value of the dt
prev_error = [0,0,0]    # surge yaw depth 
dt = [0,0,0]            # surge yaw depth 
int_error = [0,0,0]     # surge yaw depth 

#---control gains---
global kp
global ki
global kd

kp = [0,0, 5]           # surge yaw depth
kd = [0,0, 0.2]        # surge yaw depth
ki = [0,0, 0.4]          # surge yaw depth

#### control mode
global control_state #the state we want our system to go
global control_mode # the mode at which we want to control
global floatability

floatability = 11/4
###ideally the pitch should always be 0
control_state = [0,0,0]   # pitch yaw depth
control_mode = [3,3,3]      #0 for not controlling, 1 for P, 2 for PI, 3 for PID

#only use PID. the P or PI is not implemented. To use P and PI just make the
#the control gains kp/ki/kd=0

rospy.loginfo(f"we are in control mode and the control variables are \nkp={kp} ki={ki}, kd={kd}, control_mode={control_mode}")

#---------trajectory generation-------
#check the dt
# def trajectory_gen( t_final=5, mission_time=30, state_init = 0, state_final=1, dt=1/60): 
#     '''
#     t_final: the time at which we want our robot to reach final state
#     mission_time: the time upto which the robot should work
#     state_init : initial state of the robot
#     state_final: final state after control
#     dt: depends on the sensor reading

#     '''
#     t = np.linspace(0, t_final, int(t_final/dt))
#     t_steady = np.linspace(t_final, mission_time, int((mission_time-t_final)/dt))
    
#     a2 = 3*(state_final- state_init)/(t_final**2)
#     a3 = -2*(state_final-state_init)/(t_final**3)
    
#     traj = state_init + (a2 * (t**2)) + (a3 *(t**3)) 
    
#     derivative_traj = state_init + 2*(a2*t) + 3*(a3 *(t**2))
    
#     N = t_steady.shape[0] 

#     state_steady = state_final* np.ones((1, N))
#     state_steady = state_steady.squeeze()
    
#     steady_state_der = state_final* np.zeros((1, N))
#     steady_state_der = steady_state_der.squeeze()
 
#     return np.concatenate((traj, state_steady), axis =0), np.concatenate((derivative_traj, steady_state_der), axis =0) 


# def generate_trajectories():
#     global depth_p0
#     global angle_pitch_a0
#     global angle_yaw_a0

#     global control_state
#     t_final = 5
#     mission_time = 600
#     traj_pitch,der_pitch = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=angle_pitch_a0, state_final=control_state[0])  
#     traj_yaw, der_yaw = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=angle_yaw_a0, state_final=control_state[1])
#     traj_depth, der_depth = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=depth_p0, state_final=control_state[2])
    
#     global traj
#     global der_traj
#     traj = [traj_pitch, traj_yaw, traj_depth]
#     der_traj = [der_pitch, der_yaw, der_depth]

#     rospy.loginfo(f"Trajectory generated successfully")
#     rospy.loginfo(f"initial_states:{angle_pitch_a0, angle_yaw_a0, depth_p0}")
#     rospy.loginfo(f"final_states: {control_state}")
#     rospy.loginfo(f"t_final: {t_final}, mission_time:{mission_time}")

    

#---previous global variables that were given---
global arming
global set_mode

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




### trajectory
global counter
counter = 0 #to index the trajectory 
#### mapping of the thrusters. Use this before sending the singlas to the motors
def get_mapping(x, battery = 12):
    if battery==16:
        #pos
        m1 = 6.660955443377915 
        c1 = 1563.8915056025257
        #neg
        m2 = 8.556474591368378 
        c2 = 1437.5105488646345
    
    elif battery==14:
        # pos--
        m1,c1 = 7.664152101775231, 1566.2199559949108
        # neg
        m2,c2 =  9.86611712277398, 1435.8849828900563

    elif battery==12:
        # pos--
        m1,c1 = 9.26409545359083, 1536.184192041436
        # neg--
        m2,c2 = 11.876213274212256, 1464.6772068472726

    
    if x>0:
        y = m1*x+c1
    elif x<0:
        y = m2*x + c2
    elif x==0:
        y=1500

    if y>=1900:
        y = 1900
    elif y<=1100:
        y=1100

    return int(y)



# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# ---------- Functions---------------
#alpha beta filtering
class AlphaBetaFilter:
    def __init__(self,initial_depth,a=0.1,b=0.005,dt=1/58):
        self.a = a
        self.b = b
        self.dt = dt
        self.vk=0
        self.xk=0
        self.xk_0=initial_depth
        self.vk_0=0

    def filter_step(self, xm):
        self.xk = self.xk_0 + self.vk_0*self.dt
        self.vk = self.vk_0

        self.rk = xm - self.xk
        self.xk +=self.a*self.rk
        self.vk += (self.b*self.rk)/self.dt

        self.xk_0 = self.xk
        self.vk_0 = self.vk 
        
        return self.vk_0

def PID_control(kp=kp, ki=ki, kd=kd, state=[5,5,5], destination=[3,3,3], floatability=floatability):
    global int_error
    global dt
    global counter
    global der_traj
    global alpha_beta_filter
    global enable
    global imu_data
    
    #calculating the error in states
    # destination[2]=0.5
    error = np.subtract(np.array(destination), np.array(state))
    P = np.multiply(kp, error)
    print(f"error{error}")
    print(f"des:{state}")
    
    int_error+=np.multiply(error, dt)
    I = np.multiply(ki,int_error)
    # print(int_error)
    
    destination_der = [der_traj[0][counter], der_traj[1][counter], der_traj[2][counter]]
    
    der_depth = alpha_beta_filter.filter_step(state[2])
    state_der = [imu_data['pitch_rate'], imu_data['yaw_rate'], der_depth]
    # print(state_der)
    der_error = np.subtract(destination_der, state_der)
    # print(der_error)
    D = np.multiply(kd, der_error)

    DD = P+I+D +[0,0,floatability]
    # AA = np.multiply(DD,np.array([0,1,0]))
    print(DD)
    return DD 


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
        
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        rospy.loginfo("Mode correction")
        generate_trajectories()

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
    global arming
    global set_mode
    global imu_data #to store angles and their rate

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
    
    imu_data = {
        'roll':angle_roll,
        'pitch':angle_pitch,
        'yaw':angle_yaw,
        'roll_rate':p,
        'pitch_rate':q,
        'yaw_rate':r
    }
    
    
    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return

    
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
        global alpha_beta_filter
        alpha_beta_filter = AlphaBetaFilter(initial_depth=depth_p0) #filtering
        init_p0 = False

    depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0

    # setup depth servo control here
    # ...

    # control_bag.write(topic='control_depth_'+str(depth_control_mode)+'_'+str(kp)+'_'+str(ki)+'_'+str(kd), msg = control_cmd )




def mapValueScalSat(value):  # ask V 
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
	msg_override.channels[3] = np.uint(channel_yaw)        # pulseCmd[5]--> yaw
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500
	pub_msg_override.publish(msg_override)

global exploring_heading
exploring_heading = False
global counter_explore_heading
counter_explore_heading = 0
global mod
mod =5

def explore_heading(current_state):
    #lets assume the pinger on the right
    global exploring_heading
    global angle_yaw_a0
    global counter_explore_heading
    global mod 

    if current_state[0]>=120:
        exploring_heading=False
        angle_yaw_a0 = current_state[1]
        rospy.loginfo('Done exploring')

        return

    controller_signal = PID_control(
                            kp=kp,ki=ki,kd=kd,
                            state=current_state,
                            destination =[current_state[0],angle_yaw_a0+mod*counter_explore_heading ,current_state[2]], 
                            floatability=floatability
                            )
    counter_explore_heading+=1

    if np.abs(mod*counter_explore_heading)>=45:
        mod *= -1
        counter_explore_heading=0

    write_thurster_values(controller_signal)


def write_thurster_values(controller_signal):
    global control_mode
    Correction_depth = 1500	    
    Correction_surge = 1500
    Correction_yaw = 1500

    # Send PWM commands to motors
    if sum(control_mode)>0:
        Correction_surge = int(get_mapping(controller_signal[0]))
        Correction_yaw = int(get_mapping(controller_signal[1]))
        Correction_depth = int(get_mapping(controller_signal[2]))
        print(f"control signal in pwm: {Correction_surge},{Correction_yaw}, {Correction_depth}")
    
    setOverrideRCIN(1500, 1500, Correction_depth, Correction_yaw, Correction_surge, 1500)
    


def autonomous_control_wall_follow():
    #params for control
    global control_state #the state we want our system to go
    global control_mode # the mode at which we want to control
    global floatability
    global kp
    global ki
    global kd
    global counter
    global imu_data
    global traj
    global pinger_confidence
    global pinger_distance
    global prev_pinger_distance
    global angle_yaw_a0
    global exploring_heading
    global current_state

    ####edit this
    if pinger_confidence:
        current_state = [pinger_distance, imu_data['yaw'], depth_wrt_startup]
        prev_pinger_distance=pinger_distance
    
    # elif sum(control_mode)>0 and set_mode[2] and arming:
    else:
        current_state = [prev_pinger_distance, imu_data['yaw'], depth_wrt_startup]
        
    if exploring_heading:
        explore_heading(current_state=current_state)
        return

    if current_state[0]<=70:
        rospy.loginfo('distance less than 70. Now exploring')
        if sum(control_mode)>0 and set_mode[2] and arming and not exploring_heading:
            controller_signal = PID_control(
                            kp=kp,ki=ki,kd=kd,
                            state=current_state,
                            destination =[current_state[0],angle_yaw_a0 ,current_state[2]], 
                            floatability=floatability
                            )
            
            write_thurster_values(controller_signal)

            exploring_heading=True
            
            return


    else:
        if sum(control_mode)>0 and set_mode[2] and arming and not exploring_heading:
            controller_signal = PID_control(
                            kp=kp,ki=ki,kd=kd,
                            state=current_state,
                            destination =[60,angle_yaw_a0 ,current_state[2]], #check this as well
                            floatability=floatability
                            )
            
            write_thurster_values(controller_signal)
            
            return






def subscriber():
    global counter
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
    rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
    #rospy.Subscriber("/dvl/data", DVL, DvlCallback)
    rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
    autonomous_control_wall_follow()
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

