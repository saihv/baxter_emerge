#!/usr/bin/env python

import argparse
import rospy
import math 
import pickle

import numpy
from numpy import matrix
from collections import deque

import csv

import OSC
import matplotlib.pyplot as plt
import time

import baxter_interface
import baxter_external_devices
import socket

from baxter_interface import CHECK_VERSION

UDP_RECV_IP = '169.254.9.4'
UDP_RECV_PORT = 8800


socket_comm_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_comm_recv.bind((UDP_RECV_IP, UDP_RECV_PORT))


def baxter_teleop():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')

    neck = [0, 0, 0]
    clavicle = [0, 0, 0]
    shoulderLeft = [0, 0, 0]
    shoulderRight = [0, 0, 0]
    elbowLeft = [0, 0, 0]
    elbowRight = [0, 0, 0]
    wristLeft = [0, 0, 0]
    wristRight = [0, 0, 0]
    fingerLeft = [0, 0, 0]
    fingerRight = [0, 0, 0]

    a1 = deque([0] * 100000)
    ctr = 1
    fig = plt.figure()
    ax = plt.axes(xlim=(0,2000), ylim=(-180,180))   
    plt.hold(True)       

    def nx():
        neck[0] = x[2];

    def ny():
        neck[1] = x[2];

    def nz():
        neck[2] = x[2];

    def scx():
        clavicle[0] = x[2];

    def scy():
        clavicle[1] = x[2];

    def scz():
        clavicle[2] = x[2];

    def slx():
        shoulderLeft[0] = x[2];
    
    def sly():
        shoulderLeft[1] = x[2];

    def slz():
        shoulderLeft[2] = x[2];

    def srx():
        shoulderRight[0] = x[2];

    def sry():
        shoulderRight[1] = x[2];

    def srz():
        shoulderRight[2] = x[2];

    def elx():
        elbowLeft[0] = x[2];

    def ely():
        elbowLeft[1] = x[2];

    def elz():
        elbowLeft[2] = x[2];

    def erx():
        elbowRight[0] = x[2];

    def ery():
        elbowRight[1] = x[2];

    def erz():
        elbowRight[2] = x[2];

    def wlx():
        wristLeft[0] = x[2];

    def wly():
        wristLeft[1] = x[2];

    def wlz():
        wristLeft[2] = x[2];

    def wrx():
        wristRight[0] = x[2];

    def wry():
        wristRight[1] = x[2];

    def wrz():
        wristRight[2] = x[2];

    def hlx():
        fingerLeft[0] = x[2];

    def hly():
        fingerLeft[1] = x[2];

    def hlz():
        fingerLeft[2] = x[2];

    def hrx():
        fingerRight[0] = x[2];

    def hry():
        fingerRight[1] = x[2];

    def hrz():
        fingerRight[2] = x[2];

    options = { '/neck:tx': nx,
                '/neck:ty': ny,
                '/neck:tz': nz,
                '/shoulder_c:tx': scx, 
                '/shoulder_c:ty': scy, 
                '/shoulder_c:tz': scz, 
                '/shoulder_l:tx': slx,
                '/shoulder_l:ty': sly,
                '/shoulder_l:tz': slz,
                '/shoulder_r:tx': srx,
                '/shoulder_r:ty': sry,
                '/shoulder_r:tz': srz,
                '/elbow_l:tx': elx,
                '/elbow_l:ty': ely,
                '/elbow_l:tz': elz,
                '/elbow_r:tx': erx,
                '/elbow_r:ty': ery,
                '/elbow_r:tz': erz,
                '/wrist_l:tx': wlx,
                '/wrist_l:ty': wly,
                '/wrist_l:tz': wlz,
                '/wrist_r:tx': wrx,
                '/wrist_r:ty': wry,
                '/wrist_r:tz': wrz,
                '/handtip_l:tx': hlx,
                '/handtip_l:ty': hly,
                '/handtip_l:tz': hlz,
                '/handtip_r:tx': hrx,
                '/handtip_r:ty': hry,
                '/handtip_r:tz': hrz  }    

    
    plt.ion()
    plt.ylim([-180,180])
    plt.show()
    

    while not rospy.is_shutdown():
        data, addr = socket_comm_recv.recvfrom(16384)
        #print "Data received is "+ data

        completeList = OSC.decodeOSC(data)

        for x in OSC.decodeOSC(data):
            if isinstance(x, (list, tuple)):
                func = options[x[0]]
                func()

        neck = numpy.array(neck)
        clavicle = numpy.array(clavicle)
        shoulderLeft = numpy.array(shoulderLeft)
        shoulderRight = numpy.array(shoulderRight)
        elbowLeft = numpy.array(elbowLeft)
        elbowRight = numpy.array(elbowRight)
        wristLeft = numpy.array(wristLeft)
        wristRight = numpy.array(wristRight)
        fingerLeft = numpy.array(fingerLeft)
        fingerRight = numpy.array(fingerRight)

        bodyVectorRight = shoulderLeft - shoulderRight;
        bodyVectorLeft = -1 * bodyVectorRight;
        bodyVectorLong = clavicle - neck;

        humerusLeft = elbowLeft - shoulderLeft;
        humerusRight = elbowRight - shoulderRight;

        #print humerusLeft

        forearmLeft = wristLeft - elbowLeft;
        forearmRight = wristRight - elbowRight;

        handLeft = fingerLeft - wristLeft;
        handRight = fingerRight - wristRight;

        shoulderNormalLeft = numpy.cross(bodyVectorLeft, humerusLeft);
        elbowNormalLeft = numpy.cross(humerusLeft, forearmLeft);
        wristNormalLeft = numpy.cross(forearmLeft, handLeft);

        #print shoulderNormalLeft
        #print elbowNormalLeft
        #print wristNormalLeft

        dotProduct1 = numpy.dot(shoulderNormalLeft, elbowNormalLeft);
        dotProduct2 = numpy.dot(elbowNormalLeft, wristNormalLeft);
        
        dotProduct3 = numpy.dot(bodyVectorLong, humerusRight);
        dotProduct4 = numpy.dot(bodyVectorLong, humerusLeft)

        shoulderLeftPitch = numpy.arccos((dotProduct4) / (numpy.linalg.norm(bodyVectorLong) * numpy.linalg.norm(humerusLeft)));
        shoulderLeftYaw = numpy.arccos((numpy.dot(bodyVectorLeft, humerusLeft)) / (numpy.linalg.norm(bodyVectorLeft)*numpy.linalg.norm(humerusLeft)));
        shoulderLeftRoll = numpy.arccos(dotProduct1 / (numpy.linalg.norm(shoulderNormalLeft) * numpy.linalg.norm(elbowNormalLeft)));

        elbowLeftPitch = numpy.arccos(numpy.dot(humerusLeft, forearmLeft) / (numpy.linalg.norm(humerusLeft) * numpy.linalg.norm(forearmLeft)));
        elbowLeftRoll = numpy.arccos(dotProduct2 / (numpy.linalg.norm(elbowNormalLeft) * numpy.linalg.norm(wristNormalLeft)));

        wristLeftPitch = numpy.arccos(numpy.dot(forearmLeft, handLeft) / (numpy.linalg.norm(forearmLeft) * numpy.linalg.norm(handLeft)));

        shoulderNormalRight = numpy.cross(bodyVectorRight, humerusRight);
        elbowNormalRight = numpy.cross(humerusRight, forearmRight);
        wristNormalRight = numpy.cross(forearmRight, handRight);

        dotProduct1 = numpy.dot(shoulderNormalRight, elbowNormalRight);
        dotProduct2 = numpy.dot(elbowNormalRight, wristNormalRight);

        shoulderRightPitch = numpy.arccos((dotProduct3) / (numpy.linalg.norm(bodyVectorLong) * numpy.linalg.norm(humerusRight)));
        shoulderRightYaw = numpy.arccos((numpy.dot(bodyVectorRight, humerusRight)) / (numpy.linalg.norm(bodyVectorRight)*numpy.linalg.norm(humerusRight)));
        shoulderRightRoll = numpy.arccos(dotProduct1 / (numpy.linalg.norm(shoulderNormalRight) * numpy.linalg.norm(elbowNormalRight)));

        elbowRightPitch = numpy.arccos(numpy.dot(humerusRight, forearmRight) / (numpy.linalg.norm(humerusRight) * numpy.linalg.norm(forearmRight)));
        elbowRightRoll = numpy.arccos(dotProduct2 / (numpy.linalg.norm(elbowNormalRight) * numpy.linalg.norm(wristNormalRight)));

        wristRightPitch = numpy.arccos(numpy.dot(forearmRight, handRight) / (numpy.linalg.norm(forearmRight) * numpy.linalg.norm(handRight)));

        print "Left shoulder roll angle is %f"%(shoulderLeftRoll*180/3.1415)
        print "Right shoulder roll angle is %f"%(shoulderRightRoll*180/3.1415)

        print "Left elbow roll angle: %f"%(elbowLeftRoll*180/3.1415)
        print "Right elbow roll angle: %f"%(elbowRightRoll*180/3.1415)

        print "Left wrist pitch angle: %f"%(wristLeftPitch*180/3.1415)
        print "Right wrist pitch angle: %f"%(wristRightPitch*180/3.1415)

        anglesLeft = [abs(shoulderLeftYaw) -  2.35, 1.57 - abs(shoulderLeftPitch), abs(shoulderLeftRoll) - 4.71, abs(elbowLeftPitch) - 0.4, elbowLeftRoll, 2.5*wristLeftPitch]
        anglesRight = [2.35 - abs(shoulderRightYaw), 1.57 - abs(shoulderRightPitch), 4.71 - abs(shoulderRightRoll), abs(elbowRightPitch) - 0.4, elbowRightRoll - (shoulderRightRoll), 2.5*wristRightPitch]

   

        if anglesLeft[0] < -1.0:
            anglesLeft[0] = -1.0

        if anglesLeft[0] > 1.0:
            anglesLeft[0] = 1.0

        if anglesRight[0] > 1.0:
            anglesRight[0] = 1.0

        if anglesRight[0] < -1.0:
            anglesRight[0] = -1.0

        if anglesLeft[1] > 1.5:
            anglesLeft[1] = 1.5

        if anglesLeft[1] < -1.5:
            anglesLeft[1] = -1.5

        if anglesRight[1] < -1.5:
            anglesRight[1] = -1.5 

        if anglesRight[1] > 1.5:
            anglesRight[1] = 1.5 

        if anglesRight[3] > 3.1:
            anglesRight[3] = 3.1

        if anglesLeft[3] < 0.1:
            anglesLeft[3] = 0.1

        if anglesLeft[4] > 3.1:
            anglesLeft[4] = 3.1             

        #a1.append(anglesRight[2]*180/3.1415)
        #datatoplot = a1.pop()

        #ax.scatter(ctr, wristLeftPitch*180/3.1415)
        #ax.scatter(ctr, elbowLeftRoll*180/3.1415)
        #ax.scatter(ctr, shoulderLeftRoll*180/3.1415)
        #plt.draw()
        #ctr += 1



        #time.sleep(0.1)
        #plt.pause(0.001)

        print "Angles for the left arm are"
        print ["%0.2f" % i for i in anglesLeft]
        print "Angles for the right arm are"
        print ["%0.2f" % i for i in anglesRight]

        left.set_joint_positions({'left_s0':anglesLeft[0] , 'left_s1':anglesLeft[1], 'left_e0':anglesLeft[2], 'left_e1':anglesLeft[3], 'left_w0':anglesLeft[4], 'left_w1':anglesLeft[5], 'left_w2':0.0})
        right.set_joint_positions({'right_s0':anglesRight[0] , 'right_s1':anglesRight[1], 'right_e0':anglesRight[2], 'right_e1':anglesRight[3], 'right_w0':anglesRight[4], 'right_w1':anglesRight[5], 'right_w2':0.0})

     

                
def main():
    print("Initializing node... ")
    rospy.init_node("baxter_kinect_teleop")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    baxter_teleop()
    print("Done.")


if __name__ == '__main__':
    main()
