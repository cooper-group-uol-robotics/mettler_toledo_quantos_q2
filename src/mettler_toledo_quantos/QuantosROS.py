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
        global pubSample
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
        global pubSample
        headData = self.Quantos.getHeadData()
        try:
            xmldoc = minidom.parseString(headData)
            substance = xmldoc.getElementsByTagName('Substance')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Substance')) > 0 and len(xmldoc.getElementsByTagName('Substance')[0].childNodes) > 0) else ""
            LotID = xmldoc.getElementsByTagName('Lot_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Lot_ID')) > 0 and len(xmldoc.getElementsByTagName('Lot_ID')[0].childNodes) > 0) else ""
            UserID = xmldoc.getElementsByTagName('User_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('User_ID')) > 0 and len(xmldoc.getElementsByTagName('User_ID')[0].childNodes) > 0) else ""
            FillingDate = xmldoc.getElementsByTagName('Dispense_date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dispense_date')) > 0 and len(xmldoc.getElementsByTagName('Dispense_date')[0].childNodes) > 0) else ""
            ExpiryDate = xmldoc.getElementsByTagName('Exp._date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Exp._date')) > 0 and len(xmldoc.getElementsByTagName('Exp._date')[0].childNodes) > 0) else ""
            RetestDate = xmldoc.getElementsByTagName('Retest_date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Retest_date')) > 0 and len(xmldoc.getElementsByTagName('Retest_date')[0].childNodes) > 0) else ""
            mgFillWeight = xmldoc.getElementsByTagName('Content')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Content')) > 0 and len(xmldoc.getElementsByTagName('Content')[0].childNodes) > 0) else ""
            RemainingDoses = xmldoc.getElementsByTagName('Rem._dosages')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Rem._dosages')) > 0 and len(xmldoc.getElementsByTagName('Rem._dosages')[0].childNodes) > 0) else ""
            TerminalSNR = xmldoc.getElementsByTagName('Terminal_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Terminal_SNR')) > 0 and len(xmldoc.getElementsByTagName('Terminal_SNR')[0].childNodes) > 0) else ""
            BridgeSNR = xmldoc.getElementsByTagName('Bridge_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Bridge_SNR')) > 0 and len(xmldoc.getElementsByTagName('Bridge_SNR')[0].childNodes) > 0) else ""
            BalanceType = xmldoc.getElementsByTagName('Balance_Type')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Balance_Type')) > 0 and len(xmldoc.getElementsByTagName('Balance_Type')[0].childNodes) > 0) else ""
            BalanceID = xmldoc.getElementsByTagName('Balance_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Balance_ID')) > 0 and len(xmldoc.getElementsByTagName('Balance_ID')[0].childNodes) > 0) else ""
            LastCalibration = xmldoc.getElementsByTagName('Last_cal.')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Last_cal.')) > 0 and len(xmldoc.getElementsByTagName('Last_cal.')[0].childNodes) > 0) else ""
            OptionSNR = xmldoc.getElementsByTagName('Option_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Option_SNR')) > 0 and len(xmldoc.getElementsByTagName('Option_SNR')[0].childNodes) > 0) else ""
            DoseUnitSNR = xmldoc.getElementsByTagName('Dose_unit_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dose_unit_SNR')) > 0 and len(xmldoc.getElementsByTagName('Dose_unit_SNR')[0].childNodes) > 0) else ""
            ApplicationName = xmldoc.getElementsByTagName('Appl._Name')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Appl._Name')) > 0 and len(xmldoc.getElementsByTagName('Appl._Name')[0].childNodes) > 0) else ""
            DateTime = xmldoc.getElementsByTagName('Date_Time')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Date_Time')) > 0 and len(xmldoc.getElementsByTagName('Date_Time')[0].childNodes) > 0) else ""
            LevelControl = xmldoc.getElementsByTagName('Levelcontrol')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Levelcontrol')) > 0 and len(xmldoc.getElementsByTagName('Levelcontrol')[0].childNodes) > 0) else ""
            ProductionDateHead = xmldoc.getElementsByTagName('Head_prod._date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Head_prod._date')) > 0 and len(xmldoc.getElementsByTagName('Head_prod._date')[0].childNodes) > 0) else ""
            HeadType = xmldoc.getElementsByTagName('Head_type')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Head_type')) > 0 and len(xmldoc.getElementsByTagName('Head_type')[0].childNodes) > 0) else ""
            DoseHeadLimit = xmldoc.getElementsByTagName('Dose_limit')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dose_limit')) > 0 and len(xmldoc.getElementsByTagName('Dose_limit')[0].childNodes) > 0) else ""
            DosesDoneHead = xmldoc.getElementsByTagName('Dosing_counter')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dosing_counter')) > 0 and len(xmldoc.getElementsByTagName('Dosing_counter')[0].childNodes) > 0) else ""
            mgRemainingPowderHead = xmldoc.getElementsByTagName('Rem._quantity')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Rem._quantity')) > 0 and len(xmldoc.getElementsByTagName('Rem._quantity')[0].childNodes) > 0) else ""
            pubSample.publish(substance, LotID, UserID, FillingDate, ExpiryDate, RetestDate, mgFillWeight,
                            RemainingDoses, TerminalSNR, BridgeSNR, BalanceType, BalanceID, LastCalibration,
                            OptionSNR, DoseUnitSNR, ApplicationName, DateTime, LevelControl, ProductionDateHead, HeadType, DoseHeadLimit, DosesDoneHead, mgRemainingPowderHead)
            rospy.loginfo("Published Head Data")
        except:
            rospy.loginfo("Parsing XML Data Failed")

    def getSampleData(self):
        global pubSample
        sampleData = self.Quantos.getSampleData()
        try:
            xmldoc = minidom.parseString(sampleData)
            substance = xmldoc.getElementsByTagName('Substance')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Substance')) > 0 and len(xmldoc.getElementsByTagName('Substance')[0].childNodes) > 0) else ""
            LotID = xmldoc.getElementsByTagName('Lot_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Lot_ID')) > 0 and len(xmldoc.getElementsByTagName('Lot_ID')[0].childNodes) > 0) else ""
            UserID = xmldoc.getElementsByTagName('User_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('User_ID')) > 0 and len(xmldoc.getElementsByTagName('User_ID')[0].childNodes) > 0) else ""
            FillingDate = xmldoc.getElementsByTagName('Dispense_date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dispense_date')) > 0 and len(xmldoc.getElementsByTagName('Dispense_date')[0].childNodes) > 0) else ""
            ExpiryDate = xmldoc.getElementsByTagName('Exp._date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Exp._date')) > 0 and len(xmldoc.getElementsByTagName('Exp._date')[0].childNodes) > 0) else ""
            RetestDate = xmldoc.getElementsByTagName('Retest_date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Retest_date')) > 0 and len(xmldoc.getElementsByTagName('Retest_date')[0].childNodes) > 0) else ""
            mgFillWeight = xmldoc.getElementsByTagName('Content')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Content')) > 0 and len(xmldoc.getElementsByTagName('Content')[0].childNodes) > 0) else ""
            RemainingDoses = xmldoc.getElementsByTagName('Rem._dosages')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Rem._dosages')) > 0 and len(xmldoc.getElementsByTagName('Rem._dosages')[0].childNodes) > 0) else ""
            TerminalSNR = xmldoc.getElementsByTagName('Terminal_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Terminal_SNR')) > 0 and len(xmldoc.getElementsByTagName('Terminal_SNR')[0].childNodes) > 0) else ""
            BridgeSNR = xmldoc.getElementsByTagName('Bridge_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Bridge_SNR')) > 0 and len(xmldoc.getElementsByTagName('Bridge_SNR')[0].childNodes) > 0) else ""
            BalanceType = xmldoc.getElementsByTagName('Balance_Type')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Balance_Type')) > 0 and len(xmldoc.getElementsByTagName('Balance_Type')[0].childNodes) > 0) else ""
            BalanceID = xmldoc.getElementsByTagName('Balance_ID')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Balance_ID')) > 0 and len(xmldoc.getElementsByTagName('Balance_ID')[0].childNodes) > 0) else ""
            LastCalibration = xmldoc.getElementsByTagName('Last_cal.')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Last_cal.')) > 0 and len(xmldoc.getElementsByTagName('Last_cal.')[0].childNodes) > 0) else ""
            OptionSNR = xmldoc.getElementsByTagName('Option_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Option_SNR')) > 0 and len(xmldoc.getElementsByTagName('Option_SNR')[0].childNodes) > 0) else ""
            DoseUnitSNR = xmldoc.getElementsByTagName('Dose_unit_SNR')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dose_unit_SNR')) > 0 and len(xmldoc.getElementsByTagName('Dose_unit_SNR')[0].childNodes) > 0) else ""
            ApplicationName = xmldoc.getElementsByTagName('Appl._Name')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Appl._Name')) > 0 and len(xmldoc.getElementsByTagName('Appl._Name')[0].childNodes) > 0) else ""
            DateTime = xmldoc.getElementsByTagName('Date_Time')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Date_Time')) > 0 and len(xmldoc.getElementsByTagName('Date_Time')[0].childNodes) > 0) else ""
            LevelControl = xmldoc.getElementsByTagName('Levelcontrol')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Levelcontrol')) > 0 and len(xmldoc.getElementsByTagName('Levelcontrol')[0].childNodes) > 0) else ""
            ProductionDateHead = xmldoc.getElementsByTagName('Head_prod._date')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Head_prod._date')) > 0 and len(xmldoc.getElementsByTagName('Head_prod._date')[0].childNodes) > 0) else ""
            HeadType = xmldoc.getElementsByTagName('Head_type')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Head_type')) > 0 and len(xmldoc.getElementsByTagName('Head_type')[0].childNodes) > 0) else ""
            DoseHeadLimit = xmldoc.getElementsByTagName('Dose_limit')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dose_limit')) > 0 and len(xmldoc.getElementsByTagName('Dose_limit')[0].childNodes) > 0) else ""
            DosesDoneHead = xmldoc.getElementsByTagName('Dosing_counter')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Dosing_counter')) > 0 and len(xmldoc.getElementsByTagName('Dosing_counter')[0].childNodes) > 0) else ""
            mgRemainingPowderHead = xmldoc.getElementsByTagName('Rem._quantity')[0].childNodes[0].data if (len(xmldoc.getElementsByTagName('Rem._quantity')) > 0 and len(xmldoc.getElementsByTagName('Rem._quantity')[0].childNodes) > 0) else ""
            pubSample.publish(substance, LotID, UserID, FillingDate, ExpiryDate, RetestDate, mgFillWeight,
                            RemainingDoses, TerminalSNR, BridgeSNR, BalanceType, BalanceID, LastCalibration,
                            OptionSNR, DoseUnitSNR, ApplicationName, DateTime, LevelControl, ProductionDateHead, HeadType, DoseHeadLimit, DosesDoneHead, mgRemainingPowderHead)
            rospy.loginfo("Published Sample Data")
        except:
            rospy.loginfo("Parsing XML Data Failed")

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
