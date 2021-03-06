#!/usr/bin/env python3

from __future__ import print_function

import roslib; roslib.load_manifest('carsim_gazebo')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
"""
move = True

moveBindings = {
	'i':(1,0,0,0),
	'o':(1,0,0,-1),
	'j':(0,0,0,1),
	'l':(0,0,0,-1),
	'u':(1,0,0,1),
	',':(-1,0,0,0),
	'.':(-1,0,0,1),
	'm':(-1,0,0,-1),
	'O':(1,-1,0,0),
	'I':(1,0,0,0),
	'J':(0,1,0,0),
	'L':(0,-1,0,0),
	'U':(1,1,0,0),
	'<':(-1,0,0,0),
	'>':(-1,-1,0,0),
	'M':(-1,1,0,0),
	't':(0,0,1,0),
	'b':(0,0,-1,0),
}

speedBindings={
	'q':(1.1,1.1),
	'z':(.9,.9),
	'w':(1.1,1),
	'x':(.9,1),
	'e':(1,1.1),
	'c':(1,.9),
}

settings = termios.tcgetattr(sys.stdin)



rospy.init_node('teleop_twist_keyboard')
twist = Twist()
speed = rospy.get_param("~speed", 3)
turn = rospy.get_param("~turn", 30)
x = 0
y = 0
z = 0
th = 0
status = 0

pub = rospy.Publisher('carsim/cmd_vel', Twist, queue_size = 1)
pubSpeed = rospy.Publisher('carsim_speed',String ,queue_size = 1)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def readLaser(LaserInfo):
	rangesList = LaserInfo.ranges
	for i  in range(120,600):
		if (rangesList[i] < 5):
			move = False
			break
		else:
			move= True
	if((not move) and (x > 0)):
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)
	data = "km = " + str(x*speed*3.6)
	pubSpeed.publish(data)

rospy.Subscriber('/carsim/laser/scan', LaserScan, readLaser)


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break


			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except Exception as e:
		print(e)

	finally:
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


