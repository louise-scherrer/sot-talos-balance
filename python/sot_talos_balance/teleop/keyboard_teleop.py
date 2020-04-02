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

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Make Pyrene walk around!
---------------------------
Walking motion by maintaining the following keys pressed:
   u    i    o
   j    k    l

No key pressed: no motion
i,k: walking straight forward/backward
j,l: walking on the left, right side respectively
u, o: turning on itself towards anti-clockwise, clockwise respectively

Default speed: 0.1 m/s
Increase/decrease speed by pressing:
s,d: each key press brings the speed down/up by 0.1m/s respectively

Absolute speed bound: 0.2m/s

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



def vels(Vref):
    to_print = "Currently Vref order is " + str(Vref)
    return to_print

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop') # changer nom du noeuds
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=50) # first queue_size was 5


    Vx = 0.0
    Vy = 0.0
    Vtheta = 0.0
    Vref = (Vx, Vy, Vtheta)
    # current velocity = 0.0 #new

    status = 0
    count = 0
#    acc = 0.1
#    target_speed = 0
#    target_turn = 0

    control_speed = 0
    control_turn = 0




    try:
        print(msg)
        print(vels(Vref))

        while(1):
            key = getKey()
            if key in moveBindings.keys(): # dimension 3
                Vx = moveBindings[key][0] # changer  pour un truc modulable a l'avanir avec integration variation vitesse
                Vy = moveBindings[key][1]
                Vtheta = moveBindings[key][2]
                count = 0
            elif key in speedBindings.keys(): # pour l'instant j'oublie ca
                Vx = Vx + speedBindings[key][0]
                Vy = Vy + speedBindings[key][0]
                Vtheta = Vtheta + speedBindings[key][0]


#                speed = speed * speedBindings[key][0] # enlever speed
 #               turn = turn * speedBindings[key][1] # enlever turn
                count = 0

#                print(vels(speed,turn))  # enlever speed et turn
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
#            elif key == ' ' or key == 'k' : # no 'k' anymore to stop
            elif key == ' ' : # je pense que je peux enlever ca
#                x = 0 # rm x
#                th = 0 #rm th
                Vx = 0.0
                Vy = 0.0
                Vtheta = 0.0


                control_speed = 0
                control_turn = 0
            else: # si aucune entree au clavier, attendre 4 tours avant de stopper ou si Ctrl+C stopper le code
                count = count + 1
                if count > 4:
#                    x = 0
#                    th = 0
                    Vx = 0.0
                    Vy = 0.0
                    Vtheta = 0.0
                if (key == '\x03'):
                    break

# ca je pense que ca vire tout court
#            target_speed = speed * x # enlever speed et x
#            target_turn = turn * th  # enlever turn et th

# A priori je gere pas la saturation en vitesse ici, elle est geree par le pg
#            if target_speed > control_speed:
#                control_speed = min( target_speed, control_speed + 0.02 )
#            elif target_speed < control_speed:
#                control_speed = max( target_speed, control_speed - 0.02 )
#            else:
#                control_speed = target_speed

#            if target_turn > control_turn:
#                control_turn = min( target_turn, control_turn + 0.1 )
#            elif target_turn < control_turn:
#                control_turn = max( target_turn, control_turn - 0.1 )
#            else:
#                control_turn = target_turn

            twist = Twist()
#            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.linear.x = Vx; twist.linear.y = Vy; twist.linear.z = 0
#            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = Vtheta
            pub.publish(twist)

# ligne deja commentees dans le code source, utile ?
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally: # quand
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



# A quoi sert ce truc ?
#ros = Ros(robot)

#def key_pressed(
#read key
#action

#def teleop():
#    pub = rospy.Publisher('teleop_order', Vector3, queue_size=100)
#    rospy.init_node('TELEOP')
#    rate = rospy.Rate(0.1) # 1 Hz

    # je mets ca la ? j'essaie plutot rosImport dans l'appli
    #ros.rosExport.add('Vector3', 'velocityDes', 'teleop_order') #??? quel topic ? doit-il deje faire partie de dynamic graph

#    while not rospy.is_shutdown():
#        #velocityDes = (0.2,0.2,0.0)
#        velocityDes = Vector3(0.2,0.2,0.0)
#        rospy.loginfo(velocityDes)
#        pub.publish(velocityDes)
#        rate.sleep()

#if __name__ == '__main__':
#    try:
#        teleop()
#    except rospy.ROSInterruptException:
#        pass


