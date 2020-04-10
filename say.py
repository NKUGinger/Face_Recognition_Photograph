#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend


import rospy
import time
from std_msgs.msg import String
import aiml
import sys
import os
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

data_before1 = ""
data_before2 = ""
data_before3 = ""
data1 = ""
data2 = ""

stop_1 = False

def save_before(data):
    global data_before1, data_before2, data_before3
    data_before3 = data_before2
    data_before2 = data_before1
    data_before1 = data

def calculate():
    global data1
    if(data_before1 == data_before2):
        data1 = data_before1
    if(data_before2 == data_before3):
        data1 = data_before2
    if(data_before1 == data_before3):
        data1 = data_before1
    if(data_before1 != data_before2 and data_before2 != data_before3 and data_before1 != data_before3):
        data1 = "Too much noise now!"

def callback1(data):
    if stop_1 == False:
        global data2
        save_before(data.data)
        calculate()
        if(data2 != data1):
            data2 = data1
            print(data2)
            soundhandle.say(data2)
        else:
            print("I have nothing to say now !")

def callback2(data):
    global stop_1
    stop_1 = True
    print(data.data)
    soundhandle.say(data.data)
    time.sleep(7)
    soundhandle.say("5")
    time.sleep(1)
    soundhandle.say("4")
    time.sleep(1)
    soundhandle.say("3")
    time.sleep(1)
    soundhandle.say("2")
    time.sleep(1)
    soundhandle.say("1")
    time.sleep(1)
    soundhandle.say("0")
    pub.publish("take photo")
    time.sleep(1)
    soundhandle.say("I have finished it !")
    time.sleep(2)
    stop_1 = False

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('answer1', String, callback1)
    rospy.Subscriber('answer2', String, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/take_photo', String, queue_size=10)
    rospy.init_node('say', anonymous=True)
    soundhandle = SoundClient()
    soundhandle.stopAll()
    listener()
