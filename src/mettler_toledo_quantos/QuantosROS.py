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
        rospy.Subscriber("Quantos_Commands", QuantosCommand, self.callback_commands)
        # Initialize ros published for readings of temperatures and stirring values
        pub = rospy.Publisher("Quantos_Readings", QuantosResponse, queue_size=10)
        rospy.loginfo("Quantos Driver Started")

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
