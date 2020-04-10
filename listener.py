#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from opencv_apps.msg import FaceArrayStamped
import aiml

string = ""
person = ["","",""]
person_num = 0
left = [0,0,0]
right = [0,0,0]
up = [0,0,0]
down = [0,0,0]

answer = ""

# Create the kernel and learn AIML files
kernel = aiml.Kernel()
kernel.learn("/home/ginger/catkin_ws/src/opencv_apps/scripts/basic_chat.aiml")

def callback1(data):
    global string, person, person_num, left, right, up, down
    try:
        string = data.faces[3].label
        message = "Too many People ! We can only take photos of no more than three people !"
    except:
        try:
            person[2] = data.faces[2].label
            person_num = 3
        except:
            person_num = 2
            person[2] = ""
        try:
            person[1] = data.faces[1].label
        except:
            person_num = 1
            person[1] = ""
        try:
            person[0] = data.faces[0].label
        except:
            person_num = 0
            person[0] = ""
        message = "OK ! The number of people is "+str(person_num)+" !"
        for i in range(0, person_num):
            left[i] = data.faces[i].face.x - data.faces[i].face.width/2
            right[i] = 638 - data.faces[i].face.x - data.faces[i].face.width/2
            up[i] = data.faces[i].face.y - data.faces[i].face.height/2
            down[i] = 478 - data.faces[i].face.y - data.faces[i].face.height/2
            if(left[i] / right[i] < 0.2):
                message = "Too right "+data.faces[i].label+" ! Please move your head a little to the left !"
            if(left[i] / right[i] > 5):
                message = "Too left "+data.faces[i].label+" ! Please move your head a little to the right !"
            if(up[i] / down[i] < 0.2):
                message = "Too high "+data.faces[i].label+" ! Please move your computer camera a little higher or move your head a little lower !"
            if(up[i] / down[i] > 5):
                message = "Too low "+data.faces[i].label+" ! Please move your computer camera a little lower or move your head a little higher !"
            if(data.faces[i].face.width > 250):
                message = "Too close "+data.faces[i].label+" ! Please move your head away a little !"
            if(data.faces[i].face.width < 50):
                message = "Too far "+data.faces[i].label+" ! Please move your head a little closer !"
    print(message)
    pub1.publish(message)

def callback2(data):
    global answer
    answer = kernel.respond(data.data)
    print(answer)
    if len(answer) > 0:
	pub2.publish(answer)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/face_recognition/output', FaceArrayStamped, callback1)
    rospy.Subscriber('lm_data', String, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub1 = rospy.Publisher('answer1', String, queue_size=10)
    pub2 = rospy.Publisher('answer2', String, queue_size=10)
    listener()
