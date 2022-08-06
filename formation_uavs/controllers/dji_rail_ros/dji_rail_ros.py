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
import cv2
import datetime


world_x = None
world_y = None
world_next_x = None
world_next_y = None
ramp_down = False
ramp_up = False
position_x = None
position_y = None

def convert_to_pitch_roll(ex, ey, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([ex, ey], R)
    # print("ex = %f, ey = %f" % (ex, ey))
    # print(yaw)
    # print("ex_ = %f, ey_ = %f" % (exy_[0], exy_[1]))
    return exy_[0], exy_[1]

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
    global world_next_x
    global world_next_y
    global position_x
    global position_y

    global ramp_up
    global ramp_down

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
    timestep = int(robot.getBasicTimeStep()) * 2
    keyboard = Keyboard()
    keyboard.enable(timestep)
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")
    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

    # arming
    for i in range(4):
        motors[i].setPosition(float("inf"))
        motors[i].setVelocity(1.0)

    print("arming")
    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0  
    k_roll_p = 50.0
    k_pitch_p = 30.0
    target_altitude = 1.0 
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getMotor('motorname')
    #  ds = robot.getDistanceSensor('dsname')
    #  ds.enable(timestep)
    roll_disturbance = 0
    pitch_disturbance = 0
    yaw_disturbance = 0
    # target
    target_x = 0
    target_y = 0
    target_z = 30
    target_yaw = 0
    i = 0
    
    first_cycle = True
    
    fixed_power = 0
    count = 0
    
    # FRDEBUG
    last_camera_send = datetime.datetime.now().timestamp()
    
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        altitude = gps.getValues()[1]
        px = gps.getValues()[0]
        py = gps.getValues()[2]
        # print("px = %f, py = %f" %(px, py))
        roll_acceleration = gyro.getValues()[0]
        pitch_acceleration = gyro.getValues()[1]
    
        #print("x = %f, y = %f, z = %f" %(px ,py, altitude))
        #print(yaw)
        # keyboard input
        
        if first_cycle:
            world_x = px
            world_y = py
            first_cycle = False

        max_fixed_power = 500
        approach_radius = 5.0

        position_x = px
        position_y = py

        dist_to_target = math.sqrt((world_y - py)**2 + (world_x - px)**2) if world_x else 0
        if not ramp_up and not ramp_down and dist_to_target > 20:
            ramp_up = True
            ramp_down = False
            #print("ramp up and down")
        
        if ramp_down and fixed_power > 20:
            fixed_power -= 2
            #print("...ramping down")
        elif ramp_down and fixed_power <= 20:
            #print("ramped down", world_x, world_y, world_next_x, world_next_y)
            fixed_power = 0
            world_x = world_next_x
            world_y = world_next_y
            ramp_down = False
            #print("done rampign down")
        elif ramp_up and fixed_power < min(max_fixed_power - 20, (max_fixed_power / approach_radius) * dist_to_target):
            fixed_power += 2
            #print("...ramping up")
        elif ramp_up and fixed_power >= min(max_fixed_power - 20, (max_fixed_power / approach_radius) * dist_to_target):
            fixed_power = max_fixed_power
            ramp_up = False
            #print("max power")
        elif not ramp_down and not ramp_up and world_x and world_y and dist_to_target < approach_radius:
            fixed_power = (max_fixed_power / approach_radius) * dist_to_target
        else:
            fixed_power = max_fixed_power

        calc_x, calc_y = calc_world_movement(px, py)
        target_x = px + calc_x * fixed_power
        target_y = py + calc_y * fixed_power
        #print(robot.getName(), "PX", px, "PY", py, "World X", world_x, "World Y", world_y, "fixed power", fixed_power)

        # Compute the roll, pitch, yaw and vertical inputs.
        pitch_err, roll_err = convert_to_pitch_roll(px - target_x, target_y - py, yaw)
        # roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration + 1*(target_y - py)
        # pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration + 20*(px - target_x)
        roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration - roll_err
        pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration - pitch_err
        yaw_input = 0.1*(target_yaw - yaw)
        clamped_difference_altitude = np.clip(target_z - altitude + k_vertical_offset, -1.0, 1.0)
        vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)

        #print("pitch input", pitch_input, "roll input", roll_input, "yaw input", yaw_input)

        # Actuate the motors taking into consideration all the computed inputs.
        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        
        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)

        if count % 5 == 0:
            this_camera_send = datetime.datetime.now().timestamp()
            
            image = camera.getImageArray()
            
            image_np = np.array(image)
            #print("Original Shape", image_np.shape)
            image_np = np.rot90(image_np, k=1, axes=(1,0))
            image_np[:, :, [2, 0]] = image_np[:, :, [0, 2]]
            image_np = np.flip(image_np, axis=1)
            #print("Resulting Shape", image_np.shape)
            _, encoded = cv2.imencode('.png', image_np)

            b64 = base64.b64encode(encoded)
            cam_img.publish(robot.getName() + "," + str(b64))   

            robot_pos_x, robot_pos_y = mapToWorld(False, px, py)
            robot_current_position = robot.getName() + "," + str(robot_pos_x) + "," + str(robot_pos_y)
            rob_pos.publish(robot_current_position)
            
            if dist_to_target < 5 and dist_to_target > 0:
                waypoint_x, waypoint_y = mapToWorld(False, world_x, world_y)
                at_waypoint.publish(robot.getName() + "," + str(waypoint_x) + "," + str(waypoint_y))
            
        count += 1
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass
    
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
        


def convertPercentToMeters(data, ros=True):
    global world_x
    global world_y
    global world_next_x
    global world_next_y
    global position_x
    global position_y
    global ramp_down
    global ramp_up
    
    if ros == True:        
        data = data.data
        

    if str(data).lower() == "stop":
        ramp_down = True
        ramp_up = True
        world_next_x = position_x
        world_next_y = position_y
        print("Stop!")
        return

    data = str(data).split(",")
    data_x = 1 - float(data[0])
    data_y = 1 - float(data[1])  # will come in from top left, 1- adjusts to bottom left

    ramp_up = True
    ramp_down = True

    world_next_x, world_next_y = mapToWorld(True, data_x, data_y)
    return

    
if __name__ == '__main__':
    dji_controller()


# Enter here exit cleanup code.

