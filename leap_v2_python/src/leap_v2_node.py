#!/usr/bin/env python
import sys

sys.path.append("../feetech")
from scservo_sdk import *                   # Uses FTServo SDK library
import time
import numpy as np
import leap_hand_utils as lhu
import csv
import os

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016664-if00'
#PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016749-if00'
#PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016664-if00'

#PORT = "COM7"
CALIBRATION_FILE_PATH = os.path.basename(PORT)
CALIBRATION_FILE_PATH = "alignments/" + str(CALIBRATION_FILE_PATH) + ".csv"
'''
LEAP V2 Controller

This script initializes and controls the LEAP V2 dexterous robot hand.
It builds on the Feetech SCServo SDK for serial communication and supports position control, velocity reading, and current monitoring.

Key Features:
- Initializes 8 servo motors for position control with safe current limits.
- Reads calibration data from a CSV file to ensure accurate open/close positions for each tendon.
- Provides high-level control via the `write_leap()` method, where:
    - MCP side motors (IDs 0, 2, 4, 6) are controlled in radians (-1.57 to 1.57).
    - Curling motors (IDs 1, 3, 5, 7) are controlled from 0 (open) to 1 (closed).  See main readme for tendon details.
- Supports efficient group sync reads and writes for fast, synchronized motor control.
- Includes methods to read motor positions, velocities, and electrical currents.
- Automatically disables torque and closes the serial connection on shutdown for safety.

Usage:
- Designed to be used as a Python module (`LeapNode` class) or directly as a standalone test script.
- Adjust the `PORT` and calibration file path as needed for your hardware setup.
'''

class LeapNode:
    def __init__(self):
        #Read the CSV of calibration values
        self.open_calib, self.close_calib, self.isLeft = self._read_limits(CALIBRATION_FILE_PATH)
        self.limits_min_max = np.sort(np.stack([self.open_calib, self.close_calib]), axis=0)
        
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
                pass
                #print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (i, scs_model_number))
            if scs_error != 0:
                print("[ID:%03d] %s" % (i,packetHandler.getRxPacketError(scs_error)))

        self._curl_motors()  
        # Set the working mode for all of the servos to 0. Mode register is the spot to change and 0 is the actual value to send.
        # Mode 0 is current-limited position control.  1 is constant velocity control.  2 is constant current control.
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_MODE, 0)
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        #Set the current limit for the side to side motors and the up and down curling motors
        #Note register 44 is the current limit.  (Register 48 is pwm limit)  
        #The FD debug servo software UI is wrong and calls 44 PWM limit and 48 current limit.  This is incorrect as of Jan 2025.
        #I HIGHLY RECOMMEND you do not raise these values, if you do you can cause the motors to overheat and damage them!
        for scs_id in [0, 2, 4, 6]:
            packetHandler.write2ByteTxRx(scs_id, 44, 300) #mA current limit    
        for scs_id in [1, 3, 5, 7]:
            packetHandler.write2ByteTxRx(scs_id, 44, 500) #mA current limit    
        
        #Create the syncwrites.  Note Sync is faster than regular write/read 2bytetxrx for repeated writing and reading.
        packetHandler.gswPos = GroupSyncWrite(packetHandler, SCSCL_GOAL_POSITION_L, 2)    
        packetHandler.gsrPosVel = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
        packetHandler.gsrPos = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 2)
        packetHandler.gsrVel = GroupSyncRead(packetHandler, SMS_STS_PRESENT_SPEED_L, 2)
        packetHandler.gsrCurr = GroupSyncRead(packetHandler, SMS_STS_PRESENT_CURRENT_L, 2)

        self.home_pose_offset = np.zeros(8)
        #Check that the tendon motors initialized between the limits in the calibration file.  
        #It is possible they are 360 degrees off.  
        #Move the calibration numbers until we are in range.  Use this as an offset.   
        for j in range(0,3):
            curr_pos = self.read_pos(raw = True)
            done = True
            for i in (1,3,5,7):
                if (curr_pos[i]) < (self.limits_min_max[0][i] - 3.14):
                    self.home_pose_offset[i] = self.home_pose_offset[i] + 6.283
                    done = False       
                elif (curr_pos[i]) > (self.limits_min_max[1][i] + 3.14):
                    self.home_pose_offset[i] = self.home_pose_offset[i] - 6.283
                    done = False  
            #print(self.home_pose_offset)
            if done:
                break
            if j == 2:
                print("Power up and down the hand.  Something is wrong with your calibration!  If this persists, check the calibration file values and make sure the tendons are slotted properly.")
                exit()

        #Read the calibration of the MCP side motors and apply it to the home_pose_offset
        #This is because the servo horns for whatever reason don't line up with the teeth on the motors.
        #The value in parenthesis calculates the offset from 0, then the 3.14 is to subtract from the motor values to get them in the 0 range.
        self.home_pose_offset[[0,2,4,6]] = -(self.open_calib[[0,2,4,6]] - 1.5707) - 3.14159

        self.write_leap([0,0,0,0,0,0,0,0])
        # Set torque to enabled
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1)
        #set max velocity to maximum allowed by motor
        for scs_id in range(0,8):
            packetHandler.write2ByteTxRx(scs_id, SMS_STS_GOAL_SPEED_L, 32766)
        #set max acc to maximum allowed by motor
        for scs_id in range(0,8):
            packetHandler.write2ByteTxRx(scs_id, SMS_STS_ACC, 254)
    #The position you can command.  
    #For the side to side motors in position 0,2,4,6, the range is -1.57 to 1.57 radians.  
    #For the up and down curling motors in position 1,3,5,7, the range is 0 to 1, unitless where 0 is fully open and 1 is fully curled.
    def write_leap(self, position):     
        position = np.array(position)
        position[[0,2,4,6]] = np.clip(position[[0,2,4,6]], -1.57, 1.57)
        position[[1,3,5,7]] = np.clip(position[[1,3,5,7]], 0, 1)

        position[[1,3,5,7]] = lhu.scale(position[[1,3,5,7]], self.open_calib[[1,3,5,7]], self.close_calib[[1,3,5,7]])
        position = position - self.home_pose_offset
        for scs_id in range(0,8):
            pos = int(position[scs_id] * 4096 / 6.28)
            if pos < 0:
                pos = -pos - 32768
            txpacket = [self.packetHandler.scs_lobyte(pos), self.packetHandler.scs_hibyte(pos)]         
            self.packetHandler.gswPos.addParam(scs_id, txpacket)     
        scs_comm_result = self.packetHandler.gswPos.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        # Clear syncwrite parameter storage
        self.packetHandler.gswPos.clearParam()
    
    #Paramters for reading values:

    #The position for MCP side in index 0,2,4,6 returned is in radians.
    #The position for Curl in index 1,3,5,7 returned from 0 to 1.
    #The velocity is returned in RPM.
    #The current is returned in ma.

    #Each tick is the minimum resolution of the motor which is (360°/4096) = 0.087890625°
    #Home pose is 180° or 2048 ticks   
    #Register 56 and 58 is position and velocity.  Register 69 is current.

    #This reads the actual position and velocity of the motor. 
    #If you need both, use this command instead of reading pos and vel individually as it reads much faster this way!
    def read_pos_vel(self, raw = False):
        for scs_id in range(0,8):
            # Add parameter storage for SCServo#1~10 present position value
            scs_addparam_result = self.packetHandler.gsrPosVel.addParam(scs_id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)
        scs_comm_result = self.packetHandler.gsrPosVel.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        pos_list = []
        vel_list = []
        for scs_id in range(0,8):
            # Check if groupsyncread data of SCServo#1~10 is available
            scs_data_result, scs_error = self.packetHandler.gsrPosVel.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
            if scs_data_result == True:
                # Get SCServo#scs_id present position value
                scs_present_position = self.packetHandler.gsrPosVel.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                if scs_present_position > 32768:
                    scs_present_position = -scs_present_position + 32768
                pos_list.append(scs_present_position/4096 * 6.28)
                scs_present_speed = self.packetHandler.gsrPosVel.getData(scs_id, SMS_STS_PRESENT_SPEED_L, 2)
                vel_list.append(self.packetHandler.scs_tohost(scs_present_speed, 15))
                #print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.packetHandler.scs_tohost(scs_present_speed, 15)))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                continue
            if scs_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.gsrPosVel.clearParam()
        pos_list = np.array(pos_list)
        vel_list = np.array(vel_list)
        pos_list = pos_list + self.home_pose_offset
        if raw:
            return (pos_list, vel_list)
        pos_list[[1,3,5,7]] = lhu.unscale(pos_list[[1,3,5,7]], self.open_calib[[1,3,5,7]], self.close_calib[[1,3,5,7]])
        return (pos_list, vel_list)
    
    #This reads the leap position of the motor.
    def read_pos(self, raw = False):
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
            # Check if groupsyncread data of SCServo#1~10 is available
            scs_data_result, scs_error = self.packetHandler.gsrPos.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
            if scs_data_result == True:
                # Get SCServo#scs_id present position value
                scs_present_position = self.packetHandler.gsrPos.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)  
                if scs_present_position > 32768:
                    scs_present_position = -scs_present_position + 32768
                pos_list.append(scs_present_position/4096 * 6.28)
                #print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.packetHandler.scs_tohost(scs_present_speed, 15)))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                return None
            if scs_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.gsrPos.clearParam()
        pos_list = np.array(pos_list)
        pos_list = pos_list+ self.home_pose_offset
        if raw:
            return pos_list
        pos_list[[1,3,5,7]] = lhu.unscale(pos_list[[1,3,5,7]], self.open_calib[[1,3,5,7]], self.close_calib[[1,3,5,7]])
        return pos_list
    
    #This reads the actual velocity (rpm) of the motor.
    def read_vel(self):
        for scs_id in range(0,8):
            # Add parameter storage for SCServo#1~10 present position value
            scs_addparam_result = self.packetHandler.gsrVel.addParam(scs_id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

        scs_comm_result = self.packetHandler.gsrVel.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        vel_list = []
        for scs_id in range(0,8):
            # Check if groupsyncread data of SCServo#1~10 is available
            scs_data_result, scs_error = self.packetHandler.gsrVel.isAvailable(scs_id, SMS_STS_PRESENT_SPEED_L, 2)
            if scs_data_result == True:
                # Get SCServo#scs_id present position value
                scs_present_speed = self.packetHandler.gsrVel.getData(scs_id, SMS_STS_PRESENT_SPEED_L, 2)
                vel_list.append(self.packetHandler.scs_tohost(scs_present_speed, 15))
                #print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.packetHandler.scs_tohost(scs_present_speed, 15)))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                continue
            if scs_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.gsrVel.clearParam()
        return np.array(vel_list)
    
    #This reads the actual current draw (The current draw is capped via our current limit max to prevent overheating)
    def read_eff(self): 
        for scs_id in range(0,8):
            # Add parameter storage for SCServo#1~10 present position value
            scs_addparam_result = self.packetHandler.gsrCurr.addParam(scs_id)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % scs_id)
        scs_comm_result = self.packetHandler.gsrCurr.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        current_list = []
        for scs_id in range(0,8):
            # Check if groupsyncread data of SCServo#1~10 is available
            scs_data_result, scs_error = self.packetHandler.gsrCurr.isAvailable(scs_id, SMS_STS_PRESENT_CURRENT_L, 2)
            if scs_data_result == True:
                # Get SCServo#scs_id present position value
                scs_present_current = self.packetHandler.gsrCurr.getData(scs_id, SMS_STS_PRESENT_CURRENT_L, 2)
                current_list.append(self.packetHandler.scs_tohost(scs_present_current, 15))
                #print("[ID:%03d] PresPos:%d PresSpd:%d" % (scs_id, scs_present_position, self.packetHandler.scs_tohost(scs_present_speed, 15)))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                continue
            if scs_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.gsrCurr.clearParam()
        return np.array(current_list)
    
    #Read the calibration file limits for the four tendon motors
    def _read_limits(self, file_name):
        file_path = file_name
        open_calib = [] #open position
        closed_calib = [] #closed position
        try:
            with open(file_path, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',', quotechar='|')
                rows = list(reader)
                for row in rows[:-1]:
                    open_calib.append(float(row[0]))
                    closed_calib.append(float(row[1]))
                last_row = rows[-1]           # this is the line with "left" or "right"
                if str(last_row[0]) == "Right":
                    isLeft = False
                else:
                    isLeft = True
        except FileNotFoundError:
            print(f"Calibration file not found.  Make sure you run calibration first!")
            print()
        return np.array(open_calib), np.array(closed_calib), isLeft
    
    def _curl_motors(self):
        for scs_id in range(0,8):
            self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
        for scs_id in range(0,8):
            self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_MODE, 2)
        #Set the current for each motor
        if self.isLeft:
            curr = [0,125,0,-125,0,-125,0,-100]
        else:
            curr = [0,-125,0,125,0,125,0,100]
        # Set torque to enabled
        for scs_id in range(0,8):
            if curr[scs_id] < 0:
                curr[scs_id] = -32768  - curr[scs_id]
            self.packetHandler.write2ByteTxRx(scs_id, 44, curr[scs_id])
        for scs_id in [1,3,5,7]:
            self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1)
        time.sleep(0.75)
        for scs_id in [1,3,5,7]:
            self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
    
    #Close out the motors at shutdown
    def close(self):
        print("Shutting Down")      
        if not hasattr(self, 'closed'):
            time.sleep(0.03)
            self.write_leap(np.zeros(8))  
            time.sleep(1.5)
            for scs_id in range(0,8):           
                self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
            self.closed = True
    def __del__(self):
        self.close()
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

if __name__ == "__main__":
    node = LeapNode()
    i = 0
    try:
        while i != 1:          
            x = np.ones(8)*i
            x[::2] = 0
            x[6] = 0.35
            node.write_leap(x)
            if i == 0:
                time.sleep(2)
            i = i + 0.01
            if i > 0.7:
                i = 0.7
                time.sleep(4)
                i = 1
            time.sleep(0.02)
            old_time = time.time()
        i = 0
        while i != 2:          
            x = np.ones(8)*i
            x[::2] = 0
            x[6] = -0.4
            node.write_leap(x)
            if i == 0:
                time.sleep(2)
            i = i + 0.01
            if i > 0.7:
                i = 0.7
                time.sleep(40)
                x = np.zeros(8)
                node.write_leap(x)
                i = 2
            time.sleep(0.02)
            old_time = time.time()
        i = 0
        while i != 2:          
            x = np.ones(8)*i
            x[::2] = 0
            #x[6] = -0.4
            x[6] = -1.2
            x[4] = -0.3
            x[7] = x[7] - 0.1
            node.write_leap(x)
            if i == 0:
                time.sleep(2)
            i = i + 0.01
            if i > 0.7:
                i = 0.7
                time.sleep(4)
                x = np.zeros(8)
                node.write_leap(x)
                exit()
            time.sleep(0.02)
            old_time = time.time()
            #print(node.home_pose_offset)
            output = np.zeros(8)
        '''output[1] = 0
        output[3] = 0
        output[5] = 0
        output[7] = 0
        
        output[2] = 0.85
        output[4] = 0.85
        addition = 1
        state = 1
        node.write_leap(output)
        time.sleep(2)
       while True:
            if state == 1:
                output[0] = output[0] + addition * 0.05
                if output[0] > 1.0:
                    addition = -1
                if output[0] < -1.0:
                    state = 2
            elif state == 2:
                output[0] = output[0] + 0.05
                if output[0] > 0:
                    state = 3
            elif state == 3:
                output[1] = output[1] + 0.0175
                if output[1] > 0.5:
                    state = 4
                    addition = 1
            elif state == 4:
                output[0] = output[0] + addition * 0.05
                if output[0] > 1.25:
                    addition = -1
                if output[0] < -1.25:
                    state = 5
            elif state == 5:
                output[0] = output[0] + 0.05
                if output[0] > 0:
                    state = 6
            elif state == 6:
                output[1] = output[1] - 0.0175
                if output[1] < 0:
                    state = 1
                    addition = 1
                    time.sleep(2)
            node.write_leap(output)
            print(state)
            print(output[0], output[1])
            time.sleep(0.025)'''
    except KeyboardInterrupt:
        node.close()