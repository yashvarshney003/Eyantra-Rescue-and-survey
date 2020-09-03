#!/usr/bin/env python

'''

* Team ID :				#SR_7415
* Author List :			Anurag Saxena, Aman Tyagi, Yash Varshney
* Filename :			SR_7415_3_1.py
* Theme :				Survey and Rescue
* Functions :			arm, disarm, whycon_callback, roll_set_pid, pitch_set_pid, altitude_set_pid, pid
* Global Variables :	NONE

'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to current position of drone. This value is being updated each time in whycon callback
		# [x,y,z]
		
		self.drone_position = [0,0,30.2]	

		# Final co-ordinates of destination
		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2,2,20]
		

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		self.error = [0,0,0]			# To compute error in 1st step of pid function
		
		self.Perror = [0,0,0]			# To store error associated with proportional term
		self.Derror = [0,0,0]			# To store error associated with derivative term
		self.Iterm = [0,0,0]			# To store error associated with integral term
		
		self.out_roll = 0				# To store the output of pid algorithem in roll axis
		self.out_pitch = 0				# To store the output of pid algorithem in pitch axis
		self.out_throttle = 0			# To store the output of pid algorithem in throttle axis


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		
		self.Kp = [19.5, 19.5 ,60.24]
		self.Ki = [0.048, 0.048 ,0.032]
		self.Kd = [210, 210 ,319.5]

		self.previous_error = [0,0,0]			# To store the previous error values to compute Derror for derivative term


		#for low change in min values and max values
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]


		self.sample_time = 0.060 # in seconds


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=5)
		self.pub0 = rospy.Publisher('/roll_error', Float64, queue_size=10)
		self.pub1 = rospy.Publisher('/pitch_error', Float64, queue_size=10)
		self.pub2 = rospy.Publisher('/alt_error', Float64, queue_size=10)



		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)


		self.arm() # ARMING THE DRONE

'''

* Function Name :	disarm
* Input :			self or NONE
* Output :			NONE
* Logic :			setting rcAUX4 value to 1100 to disarm the drone.
* Example Call :	self.disarm()

'''

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

'''

* Function Name :	arm
* Input :			self or NONE
* Output :			NONE
* Logic :			first disarming the drone and then setting rcRoll, rcYaw, rcPitch, rcAUX4 values 1500 and rcThrottle value 1000 
					to arm the drone.
* Example Call :	self.arm()

'''
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
		print("Dorne Armed.")  #To know the exact time when the drone stared its flight


'''

* Function Name :	whycon_callback
* Input :			msg of type PoseArray
* Output :			NONE
* Logic :			callback function of /whycon/poses to store the current location of drone to self.drone_position[] variable.
* Example Call :	The function gets executed each time when /whycon node publishes /whycon/poses 

'''
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z



'''

* Function Name :	roll_set_pid
* Input :			msg of type PidTune
* Output :			NONE
* Logic :			callback function of /pid_tuning_roll to store the slider PID parameters of roll to self.Kp[0], 
					self.Ki[0], self.Kd[0] variables.
* Example Call :	This function gets executed each time when /tune_pid publishes /pid_tuning_roll 

'''
	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

'''

* Function Name :	pitch_set_pid
* Input :			msg of type PidTune
* Output :			NONE
* Logic :			callback function of /pid_tuning_pitch to store the slider PID parameters of pitch to self.Kp[1], 
					self.Ki[1], self.Kd[1] variables.
* Example Call :	This function gets executed each time when /tune_pid publishes /pid_tuning_pitch 

'''
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 0.06
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

'''

* Function Name :	altitude_set_pid
* Input :			msg of type PidTune
* Output :			NONE
* Logic :			callback function of /pid_tuning_altitude to store the slider PID parameters of altitude to self.Kp[2], 
					self.Ki[2], self.Kd[2] variables.
* Example Call :	This function gets executed each time when /tune_pid publishes /pid_tuning_altitude 

'''
	def altitude_set_pid(self,alt):
		self.altkp=alt.Kp
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3


'''

* Function Name :	pid
* Input :			self or NONE
* Output :			NONE
* Logic :			computing path distance or error by subtracting final position co-ordinates from current point co-ordinates 
					=> self.drone_position[] - self.setpoint[]

					further computing Perror for proportional term, Derror for derivative term, Iterm for integral term 
					according to the equation "output = Kp*error + Iterm + Kd*(error - previous error)"
					and Iterm according to "Iterm = (Iterm + error) * Ki"

					finding output of the equation for roll, pitch and throttle by multiplying and adding the Perror, Derror and Iterm
					with Kp, Ki, Kd respectively.

					storing output of the 3 directions equations roll, pitch, throttle in self.out_roll, self.out_pitch,
					self.out_throttle respectively.

					calculating final output by adding or subtracting output of the equations from the trimmed values.

					limiting the final output in range of cartain threshold minimum and maximum values.

					storing the errors to self.previous_error[] to calculate Derror in next iteration and updating Iterm[]

					printing current time and drone position for observations.

					publishing resultant final output and errors in roll pitch and throttle on /drone_command, /roll_error,
					/pitch_error, /alt_error respectively.

					if drone is around the setpoint within the certain range (±0.5 for roll and pitch and ±1.0 for throttle)
					print "waypoint achieved" message for observation.

* Example Call :	"self.pid()" in class and "object_name.pid()" in main via object.

'''
	def pid(self):

		#1
		self.error[0] = self.drone_position[0] - self.setpoint[0]
		self.error[1] = self.drone_position[1] - self.setpoint[1]
		self.error[2] = self.drone_position[2] - self.setpoint[2]

		#2
		self.Perror[0] = self.error[0]
		self.Perror[1] = self.error[1]
		self.Perror[2] = self.error[2]

		self.Derror[0] = self.error[0] - self.previous_error[0]
		self.Derror[1] = self.error[1] - self.previous_error[1]
		self.Derror[2] = self.error[2] - self.previous_error[2]

		self.Iterm[0] = self.Iterm[0] * self.Ki[0]
		self.Iterm[1] = self.Iterm[1] * self.Ki[1]
		self.Iterm[2] = self.Iterm[2] * self.Ki[2]

		#3
		self.out_roll = (self.Perror[0] * self.Kp[0]) + self.Iterm[0] + (self.Derror[0] * self.Kd[0])
		self.out_pitch = (self.Perror[1] * self.Kp[1]) + self.Iterm[1] + (self.Derror[1] * self.Kd[1])
		self.out_throttle = (self.Perror[2] * self.Kp[2]) + self.Iterm[2] + (self.Derror[2] * self.Kd[2])

		#4
		self.cmd.rcRoll = 1492 - self.out_roll   # (-)ve because error always be in -ve if we have to move in (+)ve x axis rcRoll >1500
		self.cmd.rcPitch = 1510 + self.out_pitch   # (+)ve because error always be in -ve if we have to move in (+)ve y axis rcPitch < 1500
		self.cmd.rcThrottle = 1530 + self.out_throttle    # (+)ve because error always be in +ve until it reaches near the point if we have to move in (+)ve axis rcThrottle >1500
		
		self.cmd.rcYaw=1498					# Trimming yaw values as drone yawing itself
		
		#6
		if self.cmd.rcRoll > self.max_values[0]:
			self.cmd.rcRoll = self.max_values[0]

		if self.cmd.rcPitch > self.max_values[1]:
			self.cmd.rcPitch = self.max_values[1]

		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]


		if self.cmd.rcRoll < self.min_values[0]:
			self.cmd.rcRoll = self.min_values[0]

		if self.cmd.rcPitch < self.min_values[1]:
			self.cmd.rcPitch = self.min_values[1]

		if self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]


		#7
		self.previous_error[0] = self.error[0]
		self.previous_error[1] = self.error[1]
		self.previous_error[2] = self.error[2]

		#8
		self.Iterm[0] = self.Iterm[0] + self.error[0]
		self.Iterm[1] = self.Iterm[1] + self.error[1]
		self.Iterm[2] = self.Iterm[2] + self.error[2]

		
		self.command_pub.publish(self.cmd)		#Publishing drone command
		self.pub0.publish(self.error[0])		#Publishing error in x-axis
		self.pub1.publish(self.error[1])		#Publishing error in y-axis
		self.pub2.publish(self.error[2])		#publishing error in z-axis
		 
		if ((self.error[0] <= 0.50 and self.error[0] >= -0.50) and (self.error[1] <= 0.50 and self.error[1] >= -0.50)  and (self.error[2] <= 1.00 and self.error[2] >= -1.00)) :
			print("waypoint achieved")





if __name__ == '__main__':

	e_drone = Edrone()  #Making object named e_drone of Edrone() class
	
	r = rospy.Rate(10) #Specifying rate in Hz based upon desired PID sampling time
	while not rospy.is_shutdown():
		e_drone.pid()		#Calling pid function in while loop untill script is running
		r.sleep()			#Thread sleep with duration defined be a frequency
