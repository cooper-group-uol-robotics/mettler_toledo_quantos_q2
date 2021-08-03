#!/usr/bin/env python

# ROS Wrapper for Serial Driver for IKA RCT Digital Heater-Stirrer
# Uses ROS to facilitate control of the hotplate and stirrer
# Made by Jakub Glowacki 27/07/2021

import rospy
from mettler_toledo_quantos.msg import QuantosResponse
from mettler_toledo_quantos.msg import QuantosCommand
from mettler_toledo_quantos.QuantosSerial import QuantosDriverSerial


class QuantosDriverROS:

    def __init__(self):
        global pub
        self.Quantos = QuantosDriverSerial()  # Create object of IKADriver class, for serial communication
        # Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("IKA_Commands", IKACommand, self.callback_commands)
        # Initialize ros published for readings of temperatures and stirring values
        pub = rospy.Publisher("IKA_Readings", IKAReading, queue_size=10)
        rate = rospy.Rate(1) #Initialize rate object for consistent timed looping
        rospy.loginfo("IKA driver started")
        while not rospy.is_shutdown(): #Whenever driver is running, loop each second polling all values and publishing them to topic
            tempPlate = self.IKA.getHotplateTemp()
            tempExt = self.IKA.getExternalTemp()
            stirSpeed = self.IKA.getStirringSpeed()
            visc = self.IKA.getViscosityTrend()
            pub.publish(float(tempPlate), float(tempExt), float(stirSpeed), float(visc))
            rate.sleep()

        # Call upon appropriate function in driver for any possible command

    def startHeat(self):
        self.IKA.startHeat()
        rospy.loginfo("Turning on Heating")

    def stopHeat(self):
        self.IKA.stopHeat()
        rospy.loginfo("Turning off Heating")

    def startStir(self):
        self.IKA.startStir()
        rospy.loginfo("Turning on Stirring")

    def stopStir(self):
        self.IKA.stopStir()
        rospy.loginfo("Turning off Stirring")

    def setStir(self, stir):
        self.IKA.setStir(stir)
        rospy.loginfo("Setting Stirring To: " + str(stir) + "RPM")

    def setHeat(self, heat):
        self.IKA.setHeat(heat)
        rospy.loginfo("Setting Heating To: " + str(heat) + "C")

    # Callback for subscriber. Calls correct function depending on command received
    def callback_commands(self, msg):
        if(msg.ika_command == msg.HEATON):
            self.startHeat()
        elif(msg.ika_command == msg.HEATOFF):
            self.stopHeat()
        elif(msg.ika_command == msg.STIRON):
            self.startStir()
        elif(msg.ika_command == msg.STIROFF):
            self.stopStir()
        elif(msg.ika_command == msg.SETSTIR):
            self.setStir(msg.ika_param)
        elif(msg.ika_command == msg.SETHEAT):
            self.setHeat(msg.ika_param)
        elif(msg.ika_command == msg.STIRAT):
            self.setStir(msg.ika_param)
            self.startStir()
        elif(msg.ika_command == msg.HEATAT):
            self.setHeat(msg.ika_param)
            self.startHeat()
        elif(msg.ika_command == msg.ALLOFF):
            self.stopHeat()
            self.stopStir()
            self.setStir(0)
            self.setHeat(0)
        else:
            rospy.loginfo("invalid command")
