#!/usr/bin/env python

# Serial Driver for Mettler Toledo Quantos Dispensing System
# Uses USB-RS232 communication to send commands and receive messages
# Made by Jakub Glowacki 03/08/2021

from csv import reader
import time
import serial
import re
import os, rospkg

class QuantosDriverSerial:
    serialCom = serial.Serial()  # Globally define serial communication

    def __init__(self, serial_port):  # Init function starts serial communication
        global serialCom
        serialCom = serial.Serial(  # Initialize serial communication object
            port=serial_port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            xonxoff=True,
            timeout=120
        )

    # Commands are defined in Quantos MT-SICS datasheet, however need to be sent over serial as
    # ASCII encoded byte arrays and must end with a carriage return and line break to
    # be recognized. Received messsages can also be decoded then to unicode string
    def setTimeout(self, timeoutS):
        global serialCom
        serialCom.timeout = timeoutS

    def catchResponse(self):
        global serialCom
        x = str(serialCom.read_until(bytearray("\r\n", "ascii")).decode('ascii'))
        if (len(x) > 3):
            if (x[-3] == "B"):
                x = str(serialCom.read_until(bytearray("\r\n", "ascii")).decode('ascii'))
        else:
            x = str(serialCom.read_until(bytearray("\r\n", "ascii")).decode('ascii'))
        try:
            xErrCode = x[-3]
        except:
            xErrCode = "NoErr"
        xErrMessage = "Unknown Response"
        #Match Responses to Error messages
        rospack = rospkg.RosPack()
        with open(os.path.join(rospack.get_path("mettler_toledo_quantos_q2"), "csv", "response.csv"), 'r') as read_obj:
            # pass the file object to reader() to get the reader object
            csv_reader = reader(read_obj)
            # Pass reader object to list() to get a list of lists
            list_of_rows = list(csv_reader)
            for i in range(len(list_of_rows)):
                if (xErrCode == list_of_rows[i][0]):
                    xErrMessage=list_of_rows[i][1]
        return xErrMessage

    def startDosing(self):
        global serialCom
        serialCom.write(bytearray("QRA 61 1\r\n", "ascii"))
        return True

    def stopDosing(self):
        global serialCom
        serialCom.write(bytearray("QRA 61 4\r\n", "ascii"))
        return True

    def getFrontDoorPos(self):
        global serialCom
        serialCom.write(bytearray("QRD 2 3 7\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        response = [int(s) for s in stringx.split() if s.isdigit()]
        if (len(response) > 3):
            return response[3]
        else:
            return 99

    def getSamplerPos(self):
        global serialCom
        serialCom.write(bytearray("QRD 2 3 8\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        response = [int(s) for s in stringx.split() if s.isdigit()]
        if (len(response) > 3):
            return response[3]
        else:
            return 99

    def getHeadData(self):
        global serialCom
        serialCom.write(bytearray("QRD 2 4 11\r\n", "ascii"))
        x = serialCom.read_until(bytearray("""</Info_head>""", "ascii"))  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        st = stringx.find("QRD 2 4 11 B")
        #en = stringx.find("QRD 2 4 11 A")
        stringx = stringx[st+14:]
        return stringx

    def getSampleData(self):
        global serialCom
        serialCom.write(bytearray("QRD 2 4 12\r\n", "ascii"))
        x = serialCom.read_until(bytearray("""</Dosing>""", "ascii"))  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        st = stringx.find("QRD 2 4 12 B")
        #en = stringx.find("QRD 2 4 12 A")
        stringx = stringx[st+14:]
        return stringx

    def moveDosingHeadPin(self, locked):
        global serialCom
        if (locked):
            lockStr = "4"
        else:
            lockStr = "3"
        serialCom.write(bytearray("QRA 60 2 " + lockStr + "\r\n", "ascii"))
        return True

    def moveFrontDoor(self, open):
        global serialCom
        if (open):
            doorStr = "3"
        else:
            doorStr = "2"
        serialCom.write(bytearray("QRA 60 7 " + doorStr + "\r\n", "ascii"))
        return True

    def moveSampler(self, position):
        global serialCom
        if (0 <= position <= 30):
            serialCom.write(bytearray("QRA 60 8 " + str(position) + "\r\n", "ascii"))
            return True
        else:
            return False

    def setTappingBeforeDosing(self, activated):
        global serialCom
        if (activated):
            tapStr = "1"
        else:
            tapStr = "0"
        serialCom.write(bytearray("QRD 1 1 1 " + tapStr + "\r\n", "ascii"))
        return True

    def setTappingWhileDosing(self, activated):
        global serialCom
        if (activated):
            tapStr = "1"
        else:
            tapStr = "0"
        serialCom.write(bytearray("QRD 1 1 2 " + tapStr + "\r\n", "ascii"))
        return True

    def setTapperIntensity(self, intensity):
        global serialCom
        if (10 <= intensity <= 100):
            serialCom.write(bytearray("QRD 1 1 3 " + str(intensity) + "\r\n", "ascii"))
            return True
        else:
            return False

    def setTapperDuration(self, duration):
        global serialCom
        if (1 <= duration <= 10):
            serialCom.write(bytearray("QRD 1 1 4 " + str(duration) + "\r\n", "ascii"))
            return True
        else:
            return False

    def setTargetValue(self, target):
        global serialCom
        if (0.10 <= target <= 250000.00):
            serialCom.write(bytearray("QRD 1 1 5 " + str(target) + "\r\n", "ascii"))
            return True
        else:
            return False

    def setTolerance(self, percentage):
        global serialCom
        if (0.1 <= percentage <= 40.0):
            serialCom.write(bytearray("QRD 1 1 6 " + str(percentage) + "\r\n", "ascii"))
            return True
        else:
            return False

    def setToleranceMode(self, overdose):
        global serialCom
        if (overdose):
            tolStr = "1"
        else:
            tolStr = "0"
        serialCom.write(bytearray("QRD 1 1 7 " + tolStr + "\r\n", "ascii"))
        return True

    def setSampleID(self, ID):
        global serialCom
        if (len(ID) <= 20):
            serialCom.write(bytearray("QRD 1 1 8 " + ID + "\r\n", "ascii"))
            return True
        else:
            return False

    def setValuePan(self):
        serialCom.write(bytearray("QRD 1 1 9 0\r\n", "ascii"))
        return True

    def setAlgorithm(self, advanced):
        global serialCom
        if (advanced):
            algStr = "1"
        else:
            algStr = "0"
        serialCom.write(bytearray("QRD 1 1 14 " + algStr + "\r\n", "ascii"))
        return True

    def setAntiStatic(self, activated):
        global serialCom
        if (activated):
            asStr = "1"
        else:
            asStr = "0"
        serialCom.write(bytearray("QRD 1 1 15 " + asStr + "\r\n", "ascii"))
        return True
