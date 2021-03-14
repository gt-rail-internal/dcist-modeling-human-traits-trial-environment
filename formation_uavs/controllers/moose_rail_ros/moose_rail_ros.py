#/usr/bin/python37

"""uav_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, InertialUnit, GPS, Compass, Gyro, Motor, Keyboard
import math
import numpy as np
import rospy 
from std_msgs.msg import String
import base64

world_x = None
world_y = None
ramp_down = False
ramp_up = False
position_x = None
position_y = None

initial_heading = False
at_target = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller

# calculates x/y going to the world x/y
def calc_world_movement(px, py):
    global world_x
    global world_y
    global ramp_down
    global ramp_up

    if world_x is None or world_y is None:
        return 0, 0
    
    angle_xy = math.atan2(world_y - py, world_x - px)

    unit_x = math.cos(angle_xy) * 0.01
    unit_y = math.sin(angle_xy) * 0.01

    return unit_x, unit_y


def dji_controller():
    global world_x
    global world_y
    global position_x
    global position_y

    global ramp_up
    global ramp_down
    
    global initial_heading
    global at_target

    # create the Robot instance.
    robot = Robot()

    # start the ROS position subscriber
    rospy.init_node(robot.getName() + '_webots_controller', anonymous=True)
    rospy.Subscriber(robot.getName() + "/set_position", String, convertPercentToMeters)
    print("Created ROS subscriber", robot.getName() + "_position")

    # create the ROS position publisher
    rob_pos = rospy.Publisher(robot.getName() + '/current_position', String, queue_size=1)
    cam_img = rospy.Publisher(robot.getName() + '/current_image', String, queue_size=1)
    at_waypoint = rospy.Publisher(robot.getName() + '/at_waypoint', String, queue_size=1)

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    keyboard = Keyboard()
    keyboard.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    left_motor_1 = robot.getDevice("left motor 1")
    left_motor_2 = robot.getDevice("left motor 2")
    left_motor_3 = robot.getDevice("left motor 3")
    left_motor_4 = robot.getDevice("left motor 4")
    right_motor_1 = robot.getDevice("right motor 1")
    right_motor_2 = robot.getDevice("right motor 2")
    right_motor_3 = robot.getDevice("right motor 3")
    right_motor_4 = robot.getDevice("right motor 4")
    motors = [left_motor_1, left_motor_2, left_motor_3, left_motor_4, right_motor_1, right_motor_2, right_motor_3, right_motor_4]
    left_motors = [left_motor_1, left_motor_2, left_motor_3, left_motor_4]
    right_motors = [right_motor_1, right_motor_2, right_motor_3, right_motor_4]
    # arming
    for i in range(8):
        motors[i].setPosition(float("inf"))
        motors[i].setVelocity(0.0)

    print("arming")
    forward_velocity = 15

    # target
    target_x = 0
    target_y = 0
    i = 0
   

    fixed_power = 0
    count = 0
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        # roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
        # pitch = imu.getRollPitchYaw()[1]
        # yaw = imu.getRollPitchYaw()[2]
        # altitude = gps.getValues()[1]
        px = gps.getValues()[0]
        py = gps.getValues()[2]
        # print("px = %f, py = %f" %(px, py))
        #roll_acceleration = gyro.getValues()[0]
        #pitch_acceleration = gyro.getValues()[1]
    
        #print("x = %f, y = %f, z = %f" %(px ,py, altitude))
        #print(yaw)
        # keyboard input
        key=keyboard.getKey()
        while(key > 0):
            if (key==Keyboard.UP):
                target_yaw += 0.01
                break
            if (key==Keyboard.LEFT):
                target_yaw -= 0.01
                break
            if (key==ord('W')):
                target_x -= 0.01
                break
            if (key==ord('X')):
                target_x += 0.01
                break
            if (key==ord('A')):
                target_y += 0.01
                break
            if (key==ord('D')):
                target_y -= 0.01
                break

        max_fixed_power = 500
        approach_radius = 5.0

        position_x = px
        position_y = py
        
        dist_to_target = math.sqrt((world_y - py)**2 + (world_x - px)**2) if world_x else 0

        
        # call the function -> world_x, world_y (don't name them that though)
        # format as a string str(world_x) + "," + str(world_y)
        # publish that string
        if count % 50 == 0:
            image = camera.getImageArray()
            buffer = str(image)
            encoded = buffer.encode()
            b64 = base64.b64encode(encoded)
            cam_img.publish(str(b64))   
            robot_pos_x, robot_pos_y = mapToWorld(False, px, py)
            robot_current_position = robot.getName() + "," + str(robot_pos_x) + "," + str(robot_pos_y)
            rob_pos.publish(robot_current_position)
            if dist_to_target < 5 and dist_to_target > 0:
                waypoint_x, waypoint_y = mapToWorld(False, world_x, world_y)
                at_waypoint.publish(robot.getName() + "," + str(waypoint_x) + "," + str(waypoint_y))

        count += 1

        
        #print(robot.getName(), "PX", px, "PY", py, "World X", world_x, "World Y", world_y, "fixed power", fixed_power)
        print("Compass Values: ", compass.getValues())
        compass_x = compass.getValues()[0]
        compass_y = compass.getValues()[1]
        compass_deg = math.atan2(compass_y, compass_x) * 180 / 3.14159265
        if world_x is None and world_y is None:
            continue

        target_angle = math.atan2(world_y - py, world_x - px) * 180 / 3.14159265
       
        heading, left_angle, right_angle = find_heading(compass_deg,target_angle)
        print("left angle", left_angle,"right angle", right_angle, "compass deg", int(compass_deg), "target_angle", int(target_angle))
        
        degrees_left = abs(int(compass_deg) - int(target_angle))
        
        if initial_heading:
            # Go Left
            if heading == "left":
                print("left")
                for left_motor in left_motors:
                    left_motor.setVelocity(0)
                for right_motor in right_motors:
                    right_motor.setVelocity(forward_velocity)
                    print(">>>>", right_motor.getVelocity())
                
            # Go Right
            elif heading == "right":
                print("right")
                for left_motor in left_motors:
                    left_motor.setVelocity(forward_velocity)
                for right_motor in right_motors:
                    right_motor.setVelocity(0)
            
            if degrees_left < 3:
                 initial_heading = False
        elif not at_target:
            # Go forward
            print("forward")
            angle_scaling = 5
            right_velocity_heading = right_angle / angle_scaling if heading == "right" else -left_angle / angle_scaling
            left_velocity_heading = -right_angle / angle_scaling if heading == "right" else left_angle / angle_scaling
            
            for left_motor in left_motors:
                left_motor.setVelocity(forward_velocity + right_velocity_heading)
            for right_motor in right_motors:
                right_motor.setVelocity(forward_velocity + left_velocity_heading)
            if dist_to_target < 3:
                at_target = True
        else:
            for motor in motors:
                motor.setVelocity(0)
               

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass
        
def find_heading(compass, target):
    abs_diff = abs(compass - target)
    left_angle = float('inf')
    right_angle = float('inf')
    if compass > target:  # was >
        left_angle = abs_diff
        right_angle = 360 - abs_diff
    else:
        left_angle = 360 - abs_diff
        right_angle = abs_diff
    #if abs(left_angle - right_angle) > 350:
    #    return "forward"
    if left_angle < right_angle:
        return "left", left_angle, right_angle
    else:
        return "right", left_angle, right_angle

    #return "forward"
    
      
def mapToWorld(forward, x, y):
    # data will be of the for ".87164,.32938", x,y
    y_scale = -89.8 - 125  # using bottom left to bottom right
    x_scale = -197 - 4.66  # using bottom left to top left
    
    adjustment_x = 4.66 #1.66 - 4.66  # using start to bottom left
    adjustment_y = 125 #0 - 125

    
    if forward:
        x, y  = y, x
        x = float(x)
        y = 1 - float(y)  # will come in from top left, 1- adjusts to bottom left
        world_x = x * x_scale + adjustment_x
        world_y = y * y_scale + adjustment_y

    else:
        map_x = (x - adjustment_x) / x_scale
        map_y = (y - adjustment_y) / y_scale
        map_x = 1 - float(map_x)
        #map_y = 1 - float(map_y)
        #world_y = 1 - float(world_y)  # will come in from top left, 1- adjusts to bottom left
        map_x, map_y  = map_y, map_x

        world_x, world_y = map_x, map_y


    return world_x, world_y
        


def convertPercentToMeters(data):
    global world_x
    global world_y
    global position_x
    global position_y
    
    global initial_heading
    global at_target

    if str(data.data).lower() == "stop":
        at_target = True
        world_x = position_x
        world_y = position_y
        return

    data = str(data.data).split(",")
    data_x = 1 - float(data[0])
    data_y = 1 - float(data[1])  # will come in from top left, 1- adjusts to bottom left

    initial_heading = True
    at_target = False

    world_x, world_y = mapToWorld(True, data_x, data_y)
    return

    
if __name__ == '__main__':
    dji_controller()

# Enter here exit cleanup code.

