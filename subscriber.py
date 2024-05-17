#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn, NavControllerOutput, Waypoint
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, WaypointPush
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry




Ready_To_QHOVER = False
Reached_Desired_Height = False

tag_pos_x = None
tag_pos_y = None

e_prev_x = 0
integral_x = 0
e_prev_y = 0
integral_y = 0







def tag_detected_drone_reached_waypoint_callback(tag_info):

	# Check to see if the plane is detecting the QR code has reached the location of last waypoint
	global Ready_To_QHOVER
	if tag_info.detections:
		rospy.loginfo("Tag detected")
		
		if tag_info.detections[0].pose.pose.pose.position.x < 0.20:
			print("First if x location passed")

			if tag_info.detections[0].pose.pose.pose.position.y > -3.00:
				print("Second if y location passed")

				Ready_To_QHOVER = True




def tag_detected_drone_reached_waypoint_call(tag_info):

	# Check to see if the plane is detecting the QR code has reached the location of last waypoint
	global Ready_To_QHOVER
	if tag_info.detections:
		rospy.loginfo("Tag detected")
		
		tag_info.detections[0].pose.pose.pose.position.x
		tag_info.detections[0].pose.pose.pose.position.y

		tag_info.detections[0].pose.pose.pose.orientation.x
		tag_info.detections[0].pose.pose.pose.orientation.y
		tag_info.detections[0].pose.pose.pose.orientation.z
		tag_info.detections[0].pose.pose.pose.orientation.w




def tag_detected_center_drone_callback(pos_info):
	# Check to see if the plane is detecting the QR code and then center using rc channel commands
	global Reached_Desired_Height
	if pos_info.pose.position:
		rospy.loginfo("Hovering in place")

		# Publish motion commands
		pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)
		motion_command = OverrideRCIn()
		motion_command.channels = [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		pub.publish(motion_command)








def PID_x(Kp, Ki, Kd):
	global e_prev_x, integral_x

	tag_detection_sub = rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, tag_data)

	if tag_pos_x != None:
		tag_detection_sub.unregister()


	#X values
		measurement_x = tag_pos_x

		e_x = -measurement_x

		p_x = Kp * e_x

		integral_x += Ki * e_x

		D_x = Kd * (e_x - e_prev_x)

		MV_x = p_x + integral_x + D_x

		e_prev_x = e_x

		return MV_x, tag_pos_x
	else:
		return None, None


def PID_y(Kp, Ki, Kd):
	global e_prev_y, integral_y

	tag_detection_sub = rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, tag_data)

	if tag_pos_y != None:
		tag_detection_sub.unregister()


	#Y values
		measurement_y = -tag_pos_y

		e_y =-3 -measurement_y

		p_y = Kp * e_y

		integral_y += Ki * e_y

		D_y = Kd * (e_y - e_prev_y)

		MV_y = p_y + integral_y + D_y

		e_prev_y = e_y

		return MV_y, tag_pos_y
	else:
		return None, None


def tag_data(tag_info):
	global tag_pos_x, tag_pos_y

	if tag_info.detections:
		
		tag_pos_x = tag_info.detections[0].pose.pose.pose.position.x
		tag_pos_y = tag_info.detections[0].pose.pose.pose.position.y






def tag_detected_hover_at_3m_callback(pos_info):
	# Check to see if the plane is detecting the QR code and if so then slowly lower to 17 units above the qr code then stop
	global Reached_Desired_Height
	if pos_info.pose.position:

		if pos_info.pose.position.z > 23.00:
			print("Drone is above 3 m")
			#Extract relevant information from override
			left_right = 1500
			forward_backward = 1500
			up_down = 1350
			yaw = 1500

			
			# Publish motion commands
			pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)
			motion_command = OverrideRCIn()
			motion_command.channels = [left_right, forward_backward, up_down, yaw, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			pub.publish(motion_command)

		if pos_info.pose.position.z < 23.00:
			print("Drone is now at 3 m")

			#Extract relevant information from override
			left_right = 1500
			forward_backward = 1500
			up_down = 1500
			yaw = 1500

			
			# Publish motion commands
			pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)
			motion_command = OverrideRCIn()
			motion_command.channels = [left_right, forward_backward, up_down, yaw, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			pub.publish(motion_command)

			Reached_Desired_Height = True




def Land():

	setmode_proxy = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
	rospy.wait_for_service("/minihawk_SIM/mavros/set_mode")

	response = setmode_proxy(custom_mode='QLAND')
	rospy.loginfo("Set mode to AUO: \n%s", response)





def TakeOff():
	setmode_proxy = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
	rospy.wait_for_service("/minihawk_SIM/mavros/set_mode")

	arming_proxy = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
	rospy.wait_for_service("/minihawk_SIM/mavros/cmd/arming")

	response = setmode_proxy(custom_mode='AUTO')
	rospy.loginfo("Set mode to AUO: \n%s", response)

	response2 = arming_proxy(True)
	rospy.loginfo("Arming set to True: \n%s", response2)




def PublishWayPoints():
	pushwaypoints_proxy = rospy.ServiceProxy('/minihawk_SIM/mavros/mission/push', WaypointPush)
	rospy.wait_for_service("/minihawk_SIM/mavros/mission/push")

	waypoint1 = Waypoint()
	waypoint1.frame = 0
	waypoint1.command = 16
	waypoint1.is_current = False
	waypoint1.autocontinue = True
	waypoint1.param1 = 0.0
	waypoint1.param2 = 0.0
	waypoint1.param3=  0.0
	waypoint1.param4 = 0.0
	waypoint1.x_lat = -35.3632583
	waypoint1.y_long = 149.1652068
	waypoint1.z_alt = 584.070007324

	waypoint2 = Waypoint()
	waypoint2.frame = 3
	waypoint2.command = 22
	waypoint2.is_current = False
	waypoint2.autocontinue = True
	waypoint2.param1 = 15.0
	waypoint2.param2 = 0.0
	waypoint2.param3=  0.0
	waypoint2.param4 = 0.0
	waypoint2.x_lat = 0.0
	waypoint2.y_long = 0.0
	waypoint2.z_alt = 30.4799995422


	waypoint3 = Waypoint()
	waypoint3.frame = 3
	waypoint3.command = 17
	waypoint3.is_current = False
	waypoint3.autocontinue = True
	waypoint3.param1 = 0.0
	waypoint3.param2 = 0.0
	waypoint3.param3=  0.0
	waypoint3.param4 = 0.0
	waypoint3.x_lat = -35.356515
	waypoint3.y_long = 149.164655
	waypoint3.z_alt = 60.9599990845

	waypoints = [waypoint1, waypoint2, waypoint3]

	response = pushwaypoints_proxy(start_index= 0, waypoints= waypoints)

	rospy.loginfo("Waypoints pushed: \n%s", response)




def MonitorTag():

	# Initially subscribe to the first callback
	tag_detection_sub = rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, tag_detected_drone_reached_waypoint_callback)

    	while not rospy.is_shutdown():
		# Check if Ready_To_QHOVER is True
		if Ready_To_QHOVER:
			# Unsubscribe from the first callback
			tag_detection_sub.unregister()
			break


	rospy.sleep(5)





	setmode_proxy = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
	rospy.wait_for_service("/minihawk_SIM/mavros/set_mode")

	response = setmode_proxy(custom_mode='QLOITER')
	rospy.loginfo("Set mode to QLOITER: \n%s", response)


	# Issue commands to land the drone on the tag
	#Land()
	new_x = 0
	new_y = 0
	value = True

	#Extract relevant information from override
	left_right = 1500
	forward_backward = 1500

			
	# Publish motion commands
	pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)
	motion_command = OverrideRCIn()


	while value:
	#X values
		MV_x, tag_pos_x = PID_x(5, .001, 5)
		if MV_x is None:
			print("MV_x is none")
			continue

		if tag_pos_x is None:
			print("tag_pos_x is none")
			continue

	#Y values
		MV_y, tag_pos_y = PID_y(2, 0.0001, 2)
		if MV_y is None:
			print("MV_y is none")
			continue

		if tag_pos_y is None:
			print("tag_pos_y is none")
			continue


		X = 1500 + MV_x
		Y = 1500 + MV_y

		if X < 1000:
			X = 1000
		elif X > 2000:
			X = 2000

		if Y < 1000:
			Y = 1000
		elif Y > 2000:
			Y = 2000

		print("VALUE OF X:", X)
		print("VALUE OF Y:", Y)
		print("X POS:", tag_pos_x)
		print("Y POS:", tag_pos_y)


		motion_command.channels = [X, Y, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		pub.publish(motion_command)
		if tag_pos_x < 0.2 and tag_pos_x > -.2:
			print("ENTERED THE X")
			if tag_pos_y < -2.5 and tag_pos_y > -3.5:
				print("ENTERED THE Y")
				value = False



	rospy.sleep(5)




	if Ready_To_QHOVER:
		# Subscribe to the second callback
		tag_detection_sub3 = rospy.Subscriber("/minihawk_SIM/mavros/local_position/pose", PoseStamped, tag_detected_hover_at_3m_callback)


	while not rospy.is_shutdown():
		if Reached_Desired_Height:
			break

	if Reached_Desired_Height:
		tag_detection_sub3.unregister()

		rospy.loginfo("Reached desired height")

		rospy.sleep(5)








if __name__ == '__main__':

	# Set up the node to perform executions
	rospy.init_node('Subscriber', anonymous=True)

	# Publish list of waypoints or txt file if possible
	PublishWayPoints()

	# Execute commands to set mode to guided, arm it, and then take off to desired altitude
	TakeOff()

	# Monitor for tag and if detected (which it will be with this setup) set the mode to land
	MonitorTag()

	Center_Drone()

	Land()









