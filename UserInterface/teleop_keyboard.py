#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w : forward
a : turn left
s : backward
d : turn right
x : stop

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'a':(0,1),
        's':(-1,0),
        'd':(0,-1),
        'x':(0,0),
        'wa':(1,1),
        'wd':(1,-1),
        'sa':(-1,1),
        'sd':(-1,-1)
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def init():
    global settings, pub, speed, turn, x, th, status

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robot_control_gui')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 0.8)
    x = 0
    th = 0
    status = 0

def cleanup():
    global settings, pub

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
def process_key(key):
    global x, th, pub, speed, turn

    if key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
    else:
        x = 0
        th = 0
        if (key == '\x03'):
            # Handle Ctrl+C
            pass

    twist = Twist()
    twist.linear.x = x * speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * turn
    pub.publish(twist)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 0.8)
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
