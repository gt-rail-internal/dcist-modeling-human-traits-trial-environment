"""uav_control controller."""
from controller import Robot, GPS, Compass, Gyro, Motor, Keyboard, Emitter, Receiver
import math
import numpy as np
import struct
import time
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
receiver = robot.getReceiver("receiver")
receiver.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
motors = [left_motor, right_motor]

# arming
left_motor.setPosition(float("inf"))
left_motor.setVelocity(0.25)
right_motor.setPosition(float("inf"))
right_motor.setVelocity(0.25)
print("arming ground")
k_vertical_thrust = 1
# target
target_z = 1
target_x = 0
target_y = 0
def convert_to_target(x, y, yaw):
    dx = 1
    dy = -1
    if yaw > 0:
        yaw = -(yaw - math.pi)
    elif yaw < 0:
        yaw = -(yaw + math.pi)
    c, s = np.cos(-yaw), np.sin(-yaw)
    R = np.array(((c, -s), (s, c)))
    dxy_ = np.matmul([dx, dy], R)
    return x + dxy_[0], y + dxy_[1]

while robot.step(timestep) != -1:

    # get data from leader
    message=receiver.getData()
    dataList=struct.unpack("5f",message)
    leader_x, learder_y, leader_z, leader_yaw, leader_yaw_input = dataList[0], dataList[1], dataList[2], dataList[3], dataList[4]
    receiver.nextPacket()
    
    target_x, target_y = convert_to_target(leader_x, learder_y, leader_yaw)
    target_z = leader_z
    px = gps.getValues()[0]
    py = gps.getValues()[2]
    
    print(px, target_x, py, target_y)
    
    # Actuate the motors taking into consideration all the computed inputs.
    angle = np.arctan((target_y - py) / (target_x - px))
    left_motor_input = k_vertical_thrust + np.cos(angle)
    right_motor_input = k_vertical_thrust + np.cos(-angle)
    
    print(left_motor_input, right_motor_input)
    
    left_motor.setVelocity(left_motor_input)
    right_motor.setVelocity(right_motor_input)

    pass

# Enter here exit cleanup code.
