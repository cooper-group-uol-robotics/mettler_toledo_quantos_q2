#!/usr/bin/env python

# ROS Wrapper for Serial Driver for Mettler Toledo Quantos dispensing system
# Uses ROS to facilitate control of various functions of dispensing system
# Made by Jakub Glowacki 03/08/2021

import rospy
from mettler_toledo_quantos.msg import QuantosResponse
from mettler_toledo_quantos.msg import QuantosCommand
from mettler_toledo_quantos.msg import QuantosSample
from mettler_toledo_quantos.QuantosSerial import QuantosDriverSerial
from xml.dom import minidom

class QuantosDriverROS:
    global doorPos
    global samplerPos
    def __init__(self):
        global pub
        global doorPos
        global samplerPos
        doorPos = 0
        samplerPos = 0
        self.Quantos = QuantosDriverSerial()  # Create object of IKADriver class, for serial communication
        # Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Quantos_Commands", QuantosCommand, self.callback_commands)
        # Initialize ros published for readings of temperatures and stirring values
        pub = rospy.Publisher("Quantos_Info", QuantosResponse, queue_size=10)
        pubSample = rospy.Publisher("Quantos_Samples", QuantosSample, queue_size=10)
        rospy.loginfo("Quantos Driver Started")

    # Call upon appropriate function in driver for any possible command
    def startDose(self):
        self.Quantos.startDosing()
        rospy.loginfo("Starting Dosing")

    def stopDose(self):
        self.Quantos.stopDosing()
        rospy.loginfo("Stopping Dosing")

    def getFrontDoorPos(self):
        doorPos = self.Quantos.getFrontDoorPos()
        pub.publish(doorPos, samplerPos)
        rospy.loginfo("Getting Door Position")

    def getSamplerPos(self):
        samplerPos = self.Quantos.getSamplerPos()
        pub.publish(doorPos, samplerPos)
        rospy.loginfo("Getting Sampler Position")

    def getHeadData(self):
        headData = self.Quantos.getHeadData()
        #fancy processing of XML stuff placeholder
        xmldoc = minidom.parseStrings(headData)
        pubSample.publish("Stuff")
        rospy.loginfo("Published Head Data")

    def getSampleData(self):
        sampleData = self.Quantos.getSampleData()
        #fancy processing of XML stuff placeholder
        xmldoc = minidom.parseStrings(sampleData)
        pubSample.publish("Stuff")
        rospy.loginfo("Published Sample Data")

    def moveDosingHeadPin(self, locked):
        self.Quantos.moveDosingHeadPin(locked)
        rospy.loginfo("Moving Dosing Head Pin")

    def moveFrontDoor(self, open):
        self.Quantos.moveFrontDoor(open)
        rospy.loginfo("Moving Front Door")

    def moveSampler(self, position):
        if (self.Quantos.moveSampler(position)):
            rospy.loginfo("Moving Sampler")
        else:
            rospy.loginfo("Sample Move Failed: Incorrect Input")

    def setTappingBeforeDosing(self, activated):
        self.Quantos.setTappingBeforeDosing(activated)
        rospy.loginfo("Setting Status of Tapping Before Dosing Setting")

    def setTappingWhileDosing(self, activated):
        self.Quantos.setTappingWhileDosing(activated)
        rospy.loginfo("Setting Status of Tapping While Dosing Setting")

    def setTapperIntensity(self, intensity):
        if(self.Quantos.setTapperIntensity(intensity)):
            rospy.loginfo("Setting Intensity of Tapper")
        else:
            rospy.loginfo("Failed to set tapper intensity: Incorrect Input")

    def setTapperDuration(self, duration):
        if(self.Quantos.setTapperDuration(duration)):
            rospy.loginfo("Setting Duration of Tapper")
        else:
            rospy.loginfo("Failed to set tapper duration: Incorrect Input")

    def setTargetValue(self, target):
        if(self.Quantos.setTargetValue(target)):
            rospy.loginfo("Setting Target Value")
        else:
            rospy.loginfo("Failed to set target value: Incorrect Input")

    def setTolerance(self, percentage):
        if(self.Quantos.setTolerance(percentage)):
            rospy.loginfo("Setting Percentage Tolerance")
        else:
            rospy.loginfo("Failed to set Tolerance: Incorrect Input")

    def setToleranceMode(self, overdose):
        self.Quantos.setToleranceMode(overdose)
        rospy.loginfo("Setting Tolerance Overdose Mode")

    def setSampleID(self, ID):
        if(self.Quantos.setSampleID(ID)):
            rospy.loginfo("Setting Sample ID")
        else:
            rospy.loginfo("Failed to set Sample ID: Incorrect Input")

    def setValuePan(self):
        self.Quantos.setValuePan()
        rospy.loginfo("Setting weighing pan as empty")

    def setAlgorithm(self, advanced):
        self.Quantos.setAlgorithm(advanced)
        rospy.loginfo("Setting dispensing algorithm value")

    def setAntiStatic(self, activated):
        self.Quantos.setAntiStatic(activated)
        rospy.loginfo("Setting antistatic system activation")

    # Callback for subscriber. Calls correct function depending on command received
    def callback_commands(self, msg):
        if(msg.quantos_command == msg.STARTDOSE):
            self.startDose()
        elif(msg.quantos_command == msg.STOPDOSE):
            self.stopDose()
        elif(msg.quantos_command == msg.GETDOORPOS):
            self.getFrontDoorPos()
        elif(msg.quantos_command == msg.GETSAMPLEPOS):
            self.getSamplerPos()
        elif(msg.quantos_command == msg.GETHEAD):
            self.getHeadData()
        elif(msg.quantos_command == msg.GETSAMPLE):
            self.getSampleData()
        elif(msg.quantos_command == msg.MOVEPIN):
            self.moveDosingHeadPin(msg.quantos_bool)
        elif(msg.quantos_command == msg.MOVEDOOR):
            self.moveFrontDoor(msg.quantos_bool)
        elif(msg.quantos_command == msg.SETSAMPLEPOS):
            self.moveSampler(msg.quantos_int)
        elif(msg.quantos_command == msg.SETTAPBEFORE):
            self.setTappingBeforeDosing(msg.quantos_bool)
        elif(msg.quantos_command == msg.SETTAPDURING):
            self.setTappingWhileDosing(msg.quantos_bool)
        elif(msg.quantos_command == msg.SETTAPINT):
            self.setTapperIntensity(msg.quantos_int)
        elif(msg.quantos_command == msg.SETTAPDURATION):
            self.setTapperDuration(msg.quantos_int)
        elif(msg.quantos_command == msg.SETTARGET):
            self.setTargetValue(msg.quantos_float)
        elif(msg.quantos_command == msg.SETTOL):
            self.setTolerance(msg.quantos_float)
        elif(msg.quantos_command == msg.SETTOLMODE):
            self.setToleranceMode(msg.quantos_bool)
        elif(msg.quantos_command == msg.SETSAMPLEID):
            self.setSampleID(msg.quantos_ID)
        elif(msg.quantos_command == msg.SETPANOFF):
            self.setValuePan()
        elif(msg.quantos_command == msg.SETALGO):
            self.setAlgorithm(msg.quantos_bool)
        elif(msg.quantos_command == msg.SETAS):
            self.setAntiStatic(msg.quantos_bool)
        else:
            rospy.loginfo("invalid command")
