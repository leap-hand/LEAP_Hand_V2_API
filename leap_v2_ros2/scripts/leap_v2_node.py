#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff

import sys
from feetech.scservo_sdk import *                   # Uses FTServo SDK library
import time
import leap_hand_utils as lhu
import csv
import os

'''
LEAP V2 Controller

This script initializes and controls the LEAP V2 dexterous robot hand.
It builds on the Feetech SCServo SDK for serial communication and supports position control, velocity reading, and current monitoring.

Key Features:
- Initializes 8 servo motors for position control with safe current limits.
- Reads calibration data from a CSV file to ensure accurate open/close positions for each tendon.
- Provides high-level control via the `write_leap()` method, where:
    - MCP side motors (IDs 0, 2, 4, 6) are controlled in radians (-1.57 to 1.57).
    - Curling motors (IDs 1, 3, 5, 7) are controlled from 0 (open) to 1 (closed).   See main readme for tendon details.
- Supports efficient group sync reads and writes for fast, synchronized motor control.
- Includes methods to read motor positions, velocities, and electrical currents.
- Automatically disables torque and closes the serial connection on shutdown for safety.

Usage:
- Designed to be used as a ROS2 module (`LeapNode` class) and copied into your ros2 workspace.
- Adjust the `PORT` and calibration file path as needed in the launch file for your hardware setup.
'''

class LeapNode(Node):
    def __init__(self):
        super().__init__('leapvtwo_node')
        # Some parameters to control the hand
        self.port = self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016668-if00').get_parameter_value().string_value
        self.calibration_file_path = os.path.basename(self.port)
        self.calibration_file_path = str(self.calibration_file_path) + ".csv"
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory("leap_v2")
        # Construct the full path
        self.calibration_file_path = os.path.join(package_share, "..", "..", "..", "..", "src", "leap_v2_ros2", "scripts", "alignments", self.calibration_file_path)
        self.calibration_file_path = os.path.normpath(self.calibration_file_path)
        print(self.calibration_file_path)
        self.open_calib, self.close_calib, self.isLeft = self._read_limits(self.calibration_file_path)
        
        self.create_subscription(JointState, 'cmd_leap', self._receive_leap, 10)
        # Creates services that can give information about the hand out
        self.create_service(LeapPosition, 'leap_position', self.pos_srv)
        self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        self.create_service(LeapEffort, 'leap_effort', self.eff_srv)
        self.create_service(LeapPosVelEff, 'leap_pos_vel', self.pos_vel_srv)
        #Init the hand
        self.portHandler = portHandler = PortHandler(self.port) #ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
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

        #Read the CSV of calibration values
        self.limits_min_max = np.sort(np.stack([self.open_calib, self.close_calib]), axis=0)
        self.home_pose_offset = np.zeros(8)
        #Check that the tendon motors initialized between the limits in the calibration file.  
        #It is possible they are 360 degrees off.  
        #Move the calibration numbers until we are in range.  Use this as an offset.   
        self.raw = True
        for j in range(0,3):
            req = LeapPosition.Request()
            res = LeapPosition.Response()
            self.pos_srv(req, res)
            curr_pos = np.array(res.position)
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
        self.raw = False
        #Read the calibration of the MCP side motors and apply it to the home_pose_offset
        #This is because the servo horns for whatever reason don't line up with the teeth on the motors.
        #The value in parenthesis calculates the offset from 0, then the 3.14 is to subtract from the motor values to get them in the 0 range.
        self.home_pose_offset[[0,2,4,6]] = -(self.open_calib[[0,2,4,6]] - 1.5707) - 3.14159
        
        # Set torque to enabled
        for scs_id in range(0,8):
            packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1)
        #set max velocity to maximum allowed by motor
        for scs_id in range(0,8):
            packetHandler.write2ByteTxRx(scs_id, SMS_STS_GOAL_SPEED_L, 32766)
        #set max acc to maximum allowed by motor
        for scs_id in range(0,8):
            packetHandler.write2ByteTxRx(scs_id, SMS_STS_ACC, 254)
        joint_state = JointState()
        joint_state.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self._receive_leap(joint_state)
    # Receive LEAP pose and directly control the robot.  Fully open here is 180 and increases in this value closes the hand.
    def _receive_leap(self, msg):
        position = msg.position
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

    # Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, request, response):
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
        if self.raw:
            response.position = pos_list.tolist()
            return response
        pos_list[[1,3,5,7]] = lhu.unscale(pos_list[[1,3,5,7]], self.open_calib[[1,3,5,7]], self.close_calib[[1,3,5,7]])
        response.position = pos_list.tolist()
        return response

    # Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, request, response):
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
        response.velocity = vel_list
        return response
    

    # Service that reads and returns the effort/current of the robot in LEAP Embodiment
    def eff_srv(self, request, response):
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
        response.effort = current_list
        return response
    #I HIGHLY RECOMMEND you use this combined service to save a lot of latency if you need position and velocity.
    def pos_vel_srv(self, request, response):
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
        vel_list = np.array(vel_list, dtype = np.float64)
        pos_list = pos_list + self.home_pose_offset
        pos_list[[1,3,5,7]] = lhu.unscale(pos_list[[1,3,5,7]], self.open_calib[[1,3,5,7]], self.close_calib[[1,3,5,7]])
        response.position = pos_list.tolist()
        response.velocity = vel_list.tolist()
        response.effort = [0.0] * len(pos_list)
        return response

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
    
    def close(self):      
        if not hasattr(self, 'closed'):
            joint_state = JointState()
            joint_state.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            self._receive_leap(joint_state)
            time.sleep(1.5)
            for scs_id in range(0,8):           
                self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
            self.closed = True
    def shutdownhook(self):
        self.close()
def main(args=None):
    rclpy.init(args=args)
    leaphand_node = LeapNode()  
    try:
        rclpy.spin(leaphand_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Call the shutdown hook before destroying the node
        leaphand_node.shutdownhook()
        leaphand_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
