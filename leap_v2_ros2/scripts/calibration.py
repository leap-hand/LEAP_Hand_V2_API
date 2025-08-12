#!/usr/bin/env python
import sys

sys.path.append("..")
from feetech.scservo_sdk import *   # Uses FTServo SDK library
import time

import numpy as np
import os
import csv
# This script is used to calibrate the LEAP Hand.
# It uses current control mode to find the curled and open position of the motors.
# Follow the print out instructions 
# The output is a CSV where each of the 4 rows is a pair of numbers.
# These are the respective motor angles required for the [open, closed] positions.

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016668-if00'
isLeft = False

class LeapCalibrate:
    def __init__(self):
        self.portHandler = portHandler = PortHandler(PORT) #ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = packetHandler = sms_sts(portHandler)
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
        for i in range(0,8):
            scs_model_number, scs_comm_result, scs_error = packetHandler.ping(i)
            if scs_comm_result != COMM_SUCCESS:
                print("[ID:%03d] %s" % (i,packetHandler.getTxRxResult(scs_comm_result)))
            else:
                print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (i, scs_model_number))
            if scs_error != 0:
                print("[ID:%03d] %s" % (i,packetHandler.getRxPacketError(scs_error)))
        # Set the working mode for all of the servos to 2. Mode register is the spot to change and 0 is the actual value to send.
        # Mode 0 is current-limited position control.  1 is constant velocity control.  2 is constant current control.
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_MODE, 2)
        #Set the current limit starting out to 0 to each motor
        for scs_id in range(0,8): 
            packetHandler.write2ByteTxRx(scs_id, 44, 0) 
        # Set torque to enabled
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1)
        #Create the syncwrite for the current controller 
        packetHandler.gsrPos = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 2)

    #This writes the desired current control to the motors
    def write_curr(self, current):     
        for scs_id in range(0,8):
            if current[scs_id] < 0:
                current[scs_id] = -32768 - current[scs_id]
            self.packetHandler.write2ByteTxRx(scs_id, 44, current[scs_id])

    #This reads the actual position of the motor in radians.
    def read_pos(self):
        for scs_id in range(0,8):
            # Add parameter storage for SCServo#1~10 present position value
            scs_addparam_result = self.packetHandler.gsrPos.addParam(scs_id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

        scs_comm_result = self.packetHandler.gsrPos.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        pos_list = []
        for scs_id in range(0,8):
            # Check if groupsyncread data of SCServo#1~8 is available
            scs_data_result, scs_error = self.packetHandler.gsrPos.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
            if scs_data_result == True:
                # Get scs_id present position value
                scs_present_position = self.packetHandler.gsrPos.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)     
                if scs_present_position >  32767:
                    scs_present_position = -scs_present_position + 32767
                pos_list.append(scs_present_position/4096 * 6.28)
                #print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.packetHandler.scs_tohost(scs_present_speed, 15)))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                continue
            if scs_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.gsrPos.clearParam()
        return np.array(pos_list)
    
if __name__ == "__main__":
    try:
        print("If you have new tendons, I suggest you run this once to wind the tendons. After that, power cycle the hand and run the script again.")
        print()
        node = LeapCalibrate()
        output = np.zeros((8,2)) 
        input("3 fingers will curl into a fully tight fist.  Please make sure nothing is in the way of the fingers and assist them to curl as tight as possible.")
        if isLeft:
            node.write_curr([0,650,0,-650,0,-650,0,0])
        else:
            node.write_curr([0,-650,0,650,0,650,0,0])
        time.sleep(5)
        pos_list = node.read_pos()
        output[[1,3,5],1] = pos_list[[1,3,5]]
        node.write_curr([0,0,0,0,0,0,0,0])
        time.sleep(4)
        for scs_id in range(0,8):
            #Disable motors and shutdown
            node.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        input("Gently open up the fingers all the way so they are in a neutral position without flexing them backwards.  Once you do, press enter to continue.")
        pos_list = node.read_pos()
        output[[1,3,5],0] = pos_list[[1,3,5]]
        input("The thumb will curl into a fully tight fist.  Please make sure nothing is in the way of the thumb.")
        if isLeft:
            node.write_curr([0,0,0,0,0,0,0,-650])  
        else:
            node.write_curr([0,0,0,0,0,0,0,650]) 
        time.sleep(5)
        pos_list = node.read_pos()
        output[7,1] = pos_list[7]
        node.write_curr([0,0,0,0,0,0,0,0])
        time.sleep(4)
        for scs_id in range(0,8):
            #Disable motors and shutdown
            node.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        input("Gently open up the thumb all the way so they are in a neutral position without flexing them backwards.  Once you do, press enter to continue.")
        pos_list = node.read_pos()
        output[7,0] = pos_list[7]
        #Make sure the angles are between 0 and 2pi
        id_t = 0
        wrap_flag = False
        while id_t < 4:
            if output[id_t,0] < 0:
                output[id_t,:] = output[id_t,:] + 6.28
                wrap_flag = True
            elif output[id_t,0] > 6.28:
                output[id_t,:] = output[id_t,:] - 6.28
                wrap_flag = True
            else:
                id_t += 1
        input("Align the 4 fingers abduction/adduction or MCP side joint to the neutral forward position.")
        pos_list = node.read_pos()
        output[[0,2,4,6],0] = pos_list[[0,2,4,6]] - 1.5707
        output[[0,2,4,6],1] = pos_list[[0,2,4,6]] + 1.5707
        print(output)
        
        if '/' in PORT:
            fp = os.path.basename(PORT)
        else:
            fp = str(input("Choose a filename to save your calibration data to."))
        
        fp = "alignments/" + str(fp) + ".csv"
        np.savetxt(fp, output, delimiter=",", fmt = '%10.5f')
        with open(fp, "a", newline="") as f:
            writer = csv.writer(f)
            if isLeft:
                writer.writerow(["Left"])
            else:
                writer.writerow(["Right"])      
        if wrap_flag:
            print()
            print("Please power cycle the hand to ensure the calibration works successfully!")
        for scs_id in range(0,8):
            #Disable motors and shutdown
            node.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        node.portHandler.closePort()  
    except KeyboardInterrupt:
        time.sleep(0.03)
        for scs_id in range(0,8):           
            node.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        time.sleep(0.5)
        node.portHandler.closePort()
        node.closed = True
