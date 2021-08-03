#!/usr/bin/env python

# ROS Wrapper for Serial Driver for Mettler Toledo Quantos dispensing system
# Uses ROS to facilitate control of various functions of dispensing system
# Made by Jakub Glowacki 03/08/2021

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
    def startDose(self):
        self.Quantos.startDosing()
        rospy.loginfo("Starting Dosing")

    def stopDose(self):
        self.Quantos.stopDosing()
        rospy.loginfo("Stopping Dosing")


    # Callback for subscriber. Calls correct function depending on command received
    def callback_commands(self, msg):
        if(msg.quantos_command == msg.STARTDOSE):
            self.startDose()
        elif(msg.quantos_command == msg.STOPDOSE):
            self.stopDose()
        else:
            rospy.loginfo("invalid command")
