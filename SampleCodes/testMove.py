import os
import time
import math
from dynamixel_sdk import *

# Control table address for Goal Position
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_TORQUE_ENABLE = 64
DXL_MOVING_STATUS_THRESHOLD = 20

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 57600
DEVICENAME = 'COM5'  # Remember to change this!

# IDs of your motors
DXL_IDS = [1, 2, 3, 4]

# Set this flag to True to use 180 degrees (CENTER_POSITIONS) as the "zero" point
# This will make the motors go to their center positions at the start
# If False, the motors will use their raw 0 value as the starting point.
USE_180_AS_ZERO = True

# Center positions in Dynamixel integer values. These are now the effective "zero" points
CENTER_POSITIONS = [2045, 1830, 1074, 2225]

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
try:
    portHandler.openPort()
except:
    print("Failed to open the port.")
    quit()

# Set port baudrate
try:
    portHandler.setBaudRate(BAUDRATE)
except:
    print("Failed to set the baudrate.")
    quit()

# Function to enable torque for all motors
def enable_torque():
    for dxl_id in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

# Function to disable torque for all motors
def disable_torque():
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

# Set initial positions based on the USE_180_AS_ZERO flag
if USE_180_AS_ZERO:
    initial_positions = CENTER_POSITIONS
else:
    initial_positions = [0] * len(DXL_IDS)

# Main loop for snake-like motion
try:
    enable_torque()
    
    # Initialize SyncWrite handler for Goal Position
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    
    # Motion loop
    start_time = time.time()
    amplitude = 399
    phase_shift = 1.57
    frequency = 0.75
    
    while True:
        current_time = time.time() - start_time
        
        for i, dxl_id in enumerate(DXL_IDS):
            # The sine wave motion now oscillates around the initial_positions
            goal_pos = int(initial_positions[i] + amplitude * math.sin(2 * math.pi * frequency * current_time + i * phase_shift))
            
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_pos)), DXL_HIBYTE(DXL_LOWORD(goal_pos)), DXL_LOBYTE(DXL_HIWORD(goal_pos)), DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            
            groupSyncWrite.addParam(dxl_id, param_goal_position)
            
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        groupSyncWrite.clearParam()
        
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("Exiting...")
finally:
    print("Resetting motors to home position and disabling torque.")
    
    # Send all motors to their home position, which is now the initial position
    for i, dxl_id in enumerate(DXL_IDS):
        goal_pos = initial_positions[i]
        
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pos)
        time.sleep(0.1) # Small delay to allow each motor to receive the command
        
    disable_torque()
    portHandler.closePort()
