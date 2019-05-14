#!/usr/bin/env python

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

import roslib
roslib.load_manifest('turtlebot_teleop')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your UGV
---------------------------
Moving around:
       w    
   a    s    d
    
z/x : increase/decrease speed

space key: force stop
CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'e':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        'q':(1,1),
        's':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'o':(1.1,1.1),
        'k':(.9,.9),
        'l':(1.1,1),
        'i':(.9,1),
        'n':(1,1.1),
        ',':(1,.9),
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

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('ugv_teleop')
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0.8
    control_turn = 0
    twist = Twist()
    try:
        print msg
        print vels(speed,turn)
        while not rospy.is_shutdown():
            key = getKey()
            
            
            if key == 'w':
               twist.linear.x = control_speed ; twist.angular.z = 0
            elif key == 's':
               twist.linear.x = -control_speed ; twist.angular.z = 0
            elif key == 'a':
               twist.angular.z = control_speed/2 ; twist.linear.x = 0
            elif key == 'd':
               twist.angular.z = -control_speed/2; twist.linear.x = 0
            elif key == 'z':
                control_speed = control_speed+ 0.2
                if control_speed > 2 : 
                    control_speed=2
                print "speed: " + control_speed

            elif key == 'x':
                control_speed=control_speed -0.2
                if control_speed < 0.4 : 
                    control_speed=0.4
                print "speed: "+ control_speed
            elif key == ' ':
                 twist.angular.z = 0 ; twist.linear.x = 0
            elif (key == '\x03'):
                break
            pub.publish(twist)


    finally:
        twist = Twist()
        twist.linear.x = 0;  twist.angular.z = 0
        twist.angular.x = 0;  twist.angular.z = 0
        pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)