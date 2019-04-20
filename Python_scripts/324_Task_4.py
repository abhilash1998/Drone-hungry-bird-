#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
#from plutoserver.msg import floatar
from plutodrone.srv import *
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

		now = int(round(time.time() * 1000))
		
		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint =  [0,0,26,0]#[-8.39,4.98,27.92,0.012]# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp =[20.28,20.26,64.50,15.8]# [18,19,60,15.8]#[553*.01,551*.01,325*.1,1535*.01]#[3.8,3.8,10,8]#[3.8,3.8,10,8]#[713*.01,701*.01,705*.01,1635*.01]#[0.075,0.0750,0.15,1]#
		self.Ki = [0.59,0.35,1.83,1.5]#[0,0,0,1.5]#[0.0075,0.0075,0.00125,0]#[0.035,0.035,0,0.070]#[0.0075,0.0075,0.00125,0]
		self.Kd = [1055.745,1055.65,637.59,600]#[500,400,400,6]#[1200*.3,1100*0.3,495*03,1057*.3]#[30,35,30,5]#[30,30,30,5]#[0.0125,0.0125,0.075,0]#



		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_values = [0,0,0,0]
		self.max_values = [1525,1525,3600,1600]
		self.min_values = [1475,1475,1200,1425]
		self.error=[0,0,0,0]
		self.previous_error=[0,0,0,0]
		self.iterm=[0,0,0,0]
		self.output=[0,0,0,0]
		self.zero_line=0
		self.no_of_pts=50
		self.path_position= [[0 for col in range(3)] for row in range(self.no_of_pts)]
		self.iter=1
		self.now=0
		self.back=0
		self.millis=0
		
		self.iter1=0
		self.at=0
		self.pre_drone_position=[0,0,0,0]
		self.drone_detected=1
		self.diffpos=[0,0,0,0]
		self.prediffpos=[0,0,0,0]
		self.path_init=0







		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		#self.sample_time = 0.060 # in seconds
		self.lastTime=0






		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Int64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Int64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Int64, queue_size=10)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Int64, queue_size=10)
		self.zero_line_pub = rospy.Publisher('/zero_line', Int16, queue_size=10)
		self.path_plan_iter=rospy.Publisher('/path_planning',Int64,queue_size=10)






		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/droneY',Float64,self.yaw_value)
		rospy.Subscriber('pid_tuning_yaw',PidTune,self.yaw_set_pid)
		#rospy.Service('PlutoService', PlutoPilot, self.yaw_value)
		rospy.Subscriber('/vrep/waypoints',PoseArray,self.path_plan)
		rospy.Subscriber('/inpt',PoseArray,self.set_point)



		#------------------------------------------------------------------------------------------------------------
		#self.disarm()
		#self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 100
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.error[2] )
		self.pitch_error_pub.publish(self.error[0])
		self.roll_error_pub.publish(self.error[1])
		self.yaw_error_pub.publish(self.error[3])
		
		rospy.sleep(0.50)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[0])
		self.roll_error_pub.publish(self.error[1])
		self.yaw_error_pub.publish(self.error[3])
		self.zero_line_pub.publish(self.zero_line)

		rospy.sleep(0.50)

	def path_plan(self,msg):
		self.path_position= [[0 for col in range(3)] for row in range(self.no_of_pts)]
		for i in range  (0,self.no_of_pts):
			self.path_position[i][0]= float("{0:.2f}".format(msg.poses[i].position.x))#"%2.f" %
			self.path_position[i][1]=float("{0:.2f}".format(msg.poses[i].position.y))#msg.poses[i].position.y
			self.path_position[i][2]= float("{0:.2f}".format(msg.poses[i].position.z))+3 #msg.poses[i].position.z
		self.path_init=self.path_init+1
		#print(self.path_position)	

	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#print(self.drone_position)
		if abs(self.drone_position[0]>=self.drone_position[1]) :
			self.drone_position[2]=self.drone_position[2]+0.25*self.drone_position[0]
		else :
			self.drone_position[2]=self.drone_position[2]+0.25*self.drone_position[1]


		
		#---------------------------------------------------------------------------------------------------------------
	def yaw_value(self,data):
		#self.drone_position[3]=data.data
		rospy.sleep(.1)
		#return PlutoPilotResponse(rcAUX2 =1500)


	def set_point(self,msg):
		if self.at==0 :

			self.setpoint[0]=float("{0:.2f}".format(msg.poses[0].position.x))
			self.setpoint[1]=float("{0:.2f}".format(msg.poses[0].position.y))
			self.setpoint[2]=float("{0:.2f}".format(msg.poses[0].position.z))
			self.at=self.at+1
			print(self.setpoint)
		#print(self.setpoint)
		

	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.1 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.01
		self.Kd[2] = alt.Kd * 03
	
		
	 
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------
	def pitch_set_pid(self,pch):
		self.Kp[0] = pch.Kp * 0.01 # This is just for an example. You can change the fraction value accordingly
		self.Ki[0] = pch.Ki * 0.01
		self.Kd[0] = pch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.01 # This is just for an example. You can change the fraction value accordingly
		self.Ki[1] = roll.Ki * 0.01
		self.Kd[1] = roll.Kd * 0.3


	def yaw_set_pid(self,yaw):
		self.Kp[3] = yaw.Kp * 0.01 # This is just for an example. You can change the fraction value accordingly
		self.Ki[3] = yaw.Ki * 0.01
		self.Kd[3] = yaw.Kd * 0.3




	def Path(self):
	 #self.drone_position=self.setpoint 	 
	 
	
	 if ( abs(self.drone_position[0]- self.setpoint[0])<=.7 
	 	) and ( abs(self.drone_position[1]- self.setpoint[1])<=0.7 ) and ( abs(self.drone_position[2]- self.setpoint[2])<=2.5 ) :
	 	 #self.iter=1
	 	 #self.iter=147
		 	
		 
		 if self.iter==1 :	 	
		 	self.iter1=1
		 	self.path_plan_iter.publish(self.iter1)
		 	rospy.sleep(0.35)
		 	#print('180')
		 	print(self.path_position)
		 	print('path1')
		 
	 	 
		 if self.iter== (self.no_of_pts-1) :
		 		
	 		self.iter1=2
	 		self.path_plan_iter.publish(self.iter1)
	 		self.disarm()
	 		rospy.sleep(.35)
	 		print(self.path_position)
	 		print('path2')

		 if self.iter== (2*(self.no_of_pts-1)) :
		 		
	 		self.iter1=3
	 		self.path_plan_iter.publish(self.iter1)
	 		rospy.sleep(.8)
	 		print(self.path_position)
	 		#self.disarm()
	 		print('path3')
	 	 if self.iter== (3*(self.no_of_pts-1)) :
	 		
	 		self.iter1=4
		 	#self.disarm()
	 		self.path_plan_iter.publish(self.iter1)
	 		rospy.sleep(.35)
	 		print(self.path_position)
	 		print('path4')
		 if self.iter== (4*(self.no_of_pts-1)) :
	
	 	    self.iter1=5
	 		#self.disarm()
	        
	 	    #self.path_plan_iter.publish(self.iter1)
	 	    print('path5')
	 	    rospy.sleep(.35)
	 	    print(self.path_position)
		 if self.iter== (5*(self.no_of_pts-1)) :
		 		
	 		self.iter1=6
	 		#self.disarm()
	 		self.path_plan_iter.publish(self.iter1)
            #rospy.sleep(.35)
			rospy.sleep(.35)
	 		print('path6')
	 		print(self.path_position)
	 	 '''if self.iter== (6*(self.no_of_pts-1)) :
		 		
		 		self.iter1=6
		 		#self.disarm()
		 		self.path_plan_iter.publish(self.iter1)
		 		rospy.sleep(.35)'''
		 
		 
		 if self.path_init==1:
			 if self.iter1 ==1 :
			 	 self.setpoint[0]=self.path_position[self.iter][0]
				 self.setpoint[1]=self.path_position[self.iter][1]	 		
				 self.setpoint[2]=self.path_position[self.iter][2]
				 self.iter=self.iter+1
				 print(self.iter)
				 print(self.path_init)
		 if self.path_init==2:
		 	 if self.iter1==2:
		 		
			 	self.setpoint[0]=self.path_position[(self.iter-self.no_of_pts+1)][0]
				self.setpoint[1]=self.path_position[(self.iter-self.no_of_pts+1)][1]	 		
				self.setpoint[2]=self.path_position[(self.iter-self.no_of_pts+1)][2]
				self.iter=self.iter+1
				print(self.iter)
				print(self.path_init)
		 if self.path_init==3:		
		 	 if self.iter1==3:
		 		#self.setpoint=[0,0,27,0]
			 	self.setpoint[0]=self.path_position[(self.iter-2*self.no_of_pts+2)][0]
				self.setpoint[1]=self.path_position[(self.iter-2*self.no_of_pts+2)][1]	 		
				self.setpoint[2]=self.path_position[(self.iter-2*self.no_of_pts+2)][2]
				self.iter=self.iter+1
				print(self.iter)
				print(self.path_init)
		 if self.path_init==4:
		 	if self.iter1==4:
		 		#self.setpoint=[0,0,27,0]
			 	self.setpoint[0]=self.path_position[(self.iter-3*self.no_of_pts+3)][0]
				self.setpoint[1]=self.path_position[(self.iter-3*self.no_of_pts+3)][1]	 		
				self.setpoint[2]=self.path_position[(self.iter-3*self.no_of_pts+3)][2]
				print(self.iter)
				print(self.path_init)
				self.iter=self.iter+1
		 if self.path_init==5:		
		 	if self.iter1==5:
		 		#self.setpoint=[0,0,27,0]
			 	self.setpoint[0]=self.path_position[(self.iter-4*self.no_of_pts+4)][0]
				self.setpoint[1]=self.path_position[(self.iter-4*self.no_of_pts+4)][1]	 		
				self.setpoint[2]=self.path_position[(self.iter-4*self.no_of_pts+4)][2]
				print(self.iter)
				print(self.path_init)
				self.iter=self.iter+1
		 if self.path_init==6:
		 	if self.iter1==6:
		 		#self.setpoint=[0,0,27,0]
			 	self.setpoint[0]=self.path_position[(self.iter-5*self.no_of_pts+5)][0]
				self.setpoint[1]=self.path_position[(self.iter-5*self.no_of_pts+5)][1]	 		
				self.setpoint[2]=self.path_position[(self.iter-5*self.no_of_pts+5)][2]
				print(self.iter)
				print(self.path_init)
				self.iter=self.iter+1
		 
	 self.pid()
		










	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------
	   SampleTime = 30
	   now = int(round(time.time() * 1000))
	   timeChange = now - self.lastTime
	   if timeChange>=SampleTime :
	    
		 #print self.drone_position 
		 for i in range(3):
			self.previous_error[i]=self.error[i]
			self.error[i] = self.setpoint[i] - self.drone_position[i]   
			self.iterm[i] = (self.iterm[i]/20 +self.error[i])*self.Ki[i]
		#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
		 	self.previous_error[3]=self.error[3]
			self.error[3] = self.setpoint[3] - self.drone_position[3]   
			self.iterm[3] = (self.iterm[i]/100 +self.error[i])*self.Ki[i]
		#	

		#throttl
		 #self.out_throttle=self.Kp[2]*(-self.error[2]) + self.Kd[2]*(-self.diffpos[2]) - self.iterm[2] 	 
		 self.out_throttle=self.Kp[2]*(-self.error[2]) + self.Kd[2]*(-self.error[2] +self.previous_error[2]) - self.iterm[2] 	
		 self.cmd.rcThrottle = 1560 + self.out_throttle 
		 #print(self.cmd.rcThrottle)	
		 if self.cmd.rcThrottle > self.max_values[2]:
		 	self.cmd.rcThrottle = self.max_values[2]

		 if self.cmd.rcThrottle < self.min_values[2]:
		 	self.cmd.rcThrottle = self.min_values[2]
			 

			 #Pitch
		 self.out_pitch=self.Kp[0]*self.error[0] + self.Kd[0]*(self.error[0] -self.previous_error[0]) + self.iterm[0]
		 #self.out_pitch=self.Kp[0]*self.error[0] + self.Kd[0]*(self.diffpos[0]) + self.iterm[0] 	
		 self.cmd.rcPitch = 1500 + self.out_pitch 

		 if self.cmd.rcPitch > self.max_values[0]:
		 	self.cmd.rcPitch = self.max_values[0]

		 if self.cmd.rcPitch < self.min_values[0]:
		 	self.cmd.rcPitch = self.min_values[0]
			 

		 #Roll
		 self.out_roll=self.Kp[1]*self.error[1] + self.Kd[1]*(self.error[1] -self.previous_error[1]) + self.iterm[1] 	
		 #self.out_roll=self.Kp[1]*self.error[1] + self.Kd[1]*(self.diffpos[1]) + self.iterm[1] 	
		 self.cmd.rcRoll = 1500 + self.out_roll 

		 if self.cmd.rcRoll > self.max_values[1]:
		 	self.cmd.rcRoll = self.max_values[1]

		 if self.cmd.rcRoll < self.min_values[1]:
		 	self.cmd.rcRoll = self.min_values[1]
			 
		 #Yaw
		 self.out_yaw=self.Kp[3]*self.error[3] + self.Kd[3]*(self.error[3] -self.previous_error[3]) + self.iterm[3] 	
		 self.cmd.rcYaw = 1500 + self.out_yaw 

		 if self.cmd.rcYaw > self.max_values[3]:
		 	self.cmd.rcYaw = self.max_values[3]

		 if self.cmd.rcYaw < self.min_values[3]:
		 	self.cmd.rcYaw = self.min_values[3]
			 

		# Steps:
		# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
		#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
		#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
		#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
		#																														self.cmd.rcPitch = self.max_values[1]
		#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
		#	8. Add error_sum





	     
		
		 for i in range(3) :
 		 	self.diffpos[i]=self.drone_position[i]-self.pre_drone_position[i]
		# if self.drone_position[0]==self.pre_drone_position[0] and self.drone_position[1]==self.pre_drone_position[1] and self.drone_position[2]==self.pre_drone_position[2]  :
		#     self.drone_detected=0
		# else :
		#	 self.drone_detected=1

		# if self.drone_detected==0 :
		#	 print('no reads')
		#	 print(self.drone_position)
		#	 print(self.pre_drone_position)
		#	 self.diffpos=self.prediffpos
		# if self.drone_detected==1 : 
		# 	 print('read')
		# print(self.drone_detected)
		 #self.prediffpos=self.diffpos




		#------------------------------------------------------------------------------------------------------------------------


			
		 self.command_pub.publish(self.cmd)
		 self.alt_error_pub.publish(self.error[2])
		 self.zero_line_pub.publish(self.zero_line)
		 self.pitch_error_pub.publish(self.error[0])
		 self.roll_error_pub.publish(self.error[1])
		 self.yaw_error_pub.publish(self.error[3])

 		 #rospy.sleep(self.sample_time)
 		 self.pre_drone_position=self.drone_position
 		 self.lastTime = now

if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		
		e_drone.Path()
