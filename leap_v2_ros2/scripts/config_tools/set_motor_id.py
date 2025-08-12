#This allows you to set the motor ID without needing a Windows machine to run the FD Servo Debug GUI.

#!/usr/bin/env python
import sys

sys.path.append("../feetech")
from scservo_sdk import *                   # Uses FTServo SDK library
import time

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
        print("Motor Found at ID:%03d.  SCServo model number : %d" % (i, scs_model_number))
    if scs_error != 0:
        pass
        #print("[ID:%03d] %s" % (i,packetHandler.getRxPacketError(scs_error)))

if len(motor_ids) > 1:
    print("Too many motors found! Exiting...")
    print(print(motor_ids))
    exit()
if len(motor_ids) == 0:
    print("No motors found! Exiting...")
    exit()
print()
id = int(input("What would you like to set the new motor ID to?"))
packetHandler.write1ByteTxRx(motor_ids[0], 5, id) #5 is the ID register
scs_model_number, scs_comm_result, scs_error = packetHandler.ping(id)
if scs_comm_result != COMM_SUCCESS:
    print("Failed First time trying again")
    packetHandler.write1ByteTxRx(motor_ids[0], 5, id) #5 is the ID register
    scs_model_number, scs_comm_result, scs_error = packetHandler.ping(id)
    if scs_comm_result != COMM_SUCCESS:
        print("ID Change Failed second time, error?")
        print("[ID:%03d] %s" % (i,packetHandler.getTxRxResult(scs_comm_result)))
    else:
        print("Motor ID successfully set to %d" % (id))
else:
    print("Motor ID successfully set to %d" % (id))