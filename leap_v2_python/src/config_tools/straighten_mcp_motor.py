#!/usr/bin/env python   
import sys

sys.path.append("../../")
from feetech.scservo_sdk import *   # Uses FTServo SDK library
import time

import numpy as np
#When you are assembling the MCP side motors, you want the horn to line up with the home position.
#However, it is not marked anywhere so it is useful to power the motor up and set it to the home pose.
#This allows you to straighten the motor without needing a Windows machine to run the FD Servo Debug GUI.

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016668-if00'

portHandler = PortHandler(PORT) #ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()
# Set port baudrate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()
# Try to ping all the servos and check that they're there
motor_ids = []
for i in range(0,8):
    scs_model_number, scs_comm_result, scs_error = packetHandler.ping(i)
    if scs_comm_result != COMM_SUCCESS:
        pass
        #print("[ID:%03d] %s" % (i,packetHandler.getTxRxResult(scs_comm_result)))
    else:
        motor_ids.append(i)
        #print("Motor Found at ID:%03d.  SCServo model number : %d" % (i, scs_model_number))
    if scs_error != 0:
        pass
        #print("[ID:%03d] %s" % (i,packetHandler.getRxPacketError(scs_error)))
print("Motor IDs found: ", motor_ids)
print()
print("Which motor would you like to set to forward?")
scs_id = int(input("Enter an integer and only do one Motor at a time."))
packetHandler.write1ByteTxRx(scs_id, SMS_STS_MODE, 0)
#Set the current limit starting out to 500mA to each motor
packetHandler.write2ByteTxRx(scs_id, 44, 200) #500mA current limit    
# Set torque to enabled
packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1)
#set max velocity to maximum allowed by motor
packetHandler.write2ByteTxRx(scs_id, SMS_STS_GOAL_SPEED_L, 32766)
#set max acc to maximum allowed by motor
packetHandler.write2ByteTxRx(scs_id, SMS_STS_ACC, 254)
packetHandler.write2ByteTxRx(scs_id, 42, 2048) #2048 is the home position
print()
input("Press Enter to turn off the motor")
packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
portHandler.closePort()
exit()