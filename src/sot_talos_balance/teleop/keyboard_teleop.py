#!/usr/bin/env python

# March 2020
# Keyboard Teleoperation of Talos, pre-step to Joytick teleoperation
# Meant to be launched with the test_joystick_dcmZmpControl_file.py
# Based (strongly) on the turtlebot teleop key program
#https://github.com/turtlebot/turtlebot/blob/melodic/turtlebot_teleop/scripts/turtlebot_teleop_key



#Detect keyboard orders
#Transform said orders in velocity command to the robot
#Publish the order
#robot.pg.velocitydes.value




# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from numpy import sign

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Walk Pyrene around!
---------------------------
Walking motion induces by pressing some of the following keys:
   u    i    o
   j    k    l

i,k: walking straight forward/backward (along the x axis in the simulation frame)
j,l: walking on the left, right side respectively (y axis)
u, o: turning on itself anti-clockwise, clockwise respectively (around z axis)

Default speed: 0.1 m/s
Increase/decrease speed by pressing:
s,d: each key press brings any non-zero value of linear or angular speed down/up by 0.1m/s respectively

Press 'Space' in order to stop the robot.

Absolute speed bound implemented in the pattern generator: close 0.2m/s

Speed vector in the form: (Vx, Vy, Vtheta). A null vector brings the robot to a stop.

CTRL-C to quit the teleoperation
"""

moveBindings = { # orders
        'i':(0.1,0.0,0.0), # straight forward
        'j':(0.0,-0.1,0.0), # walk on the left
        'l':(0.0,0.1,0.0), # walk on the right
        'k':(-0.1,0.0,0.0), # walk back straight
        'u':(0.0,0.0,-0.1), # turn to the left
        'o':(0.0,0.0,0.1), # turn to the right
           }


speedBindings = { # order to
        's':(-0.1), # bring speed down by 0.1m/s
        'd':(0.1), # bring speed up by 0.1 m/s
           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def vels(Vx, Vy, Vtheta):
    return "Currently the velocity order is (Vx, Vy, Vtheta)=%s" % ((Vx, Vy, Vtheta),)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('pyrene_teleop')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5) # first queue_size was 5


    Vx = 0.0
    Vy = 0.0
    Vtheta = 0.0
    Vref = (Vx, Vy, Vtheta)


    status = 0
    count = 0


    try:
        print(msg)
        print(vels(Vx, Vy, Vtheta))

        while(1):
            key = getKey()
            if key in moveBindings.keys(): # dimension 3
                Vx = round(Vx + moveBindings[key][0],1)
                Vy = round(Vy + moveBindings[key][1],1)
                Vtheta = round(Vtheta + moveBindings[key][2],1)
                count = 0
                print(vels(Vx, Vy, Vtheta))
            elif key in speedBindings.keys():
                if Vx != 0.0 :
                    Vx = round(Vx + sign(Vx)*speedBindings[key],1) # keeping speed values with 0.1 precision
                    # sign() handles the acceleration/deceleration of both positive and negative speeds
                if Vy != 0.0:
                    Vy = round(Vy + sign(Vy)*speedBindings[key],1)
                if Vtheta != 0.0:
                    Vtheta = round(Vtheta + sign(Vtheta)*speedBindings[key],1)

                count = 0

                print(vels(Vx, Vy, Vtheta))

                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            elif key == ' ' : # space key to stop
                Vx = 0.0
                Vy = 0.0
                Vtheta = 0.0

                print(vels(Vx, Vy, Vtheta))

            else:
                if (key == '\x03'):
                    break

# No handling of speed saturation for now, as it is handled by the pattern generator.

            twist = Twist()
            twist.linear.x = Vx; twist.linear.y = Vy; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = Vtheta
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
