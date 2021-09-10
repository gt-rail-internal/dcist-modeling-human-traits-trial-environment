from flask import Flask, render_template, request, make_response, jsonify

import rospy
from std_msgs.msg import String
import base64
import threading
import cv2
import numpy as np
import json
import datetime
import random

import sys
sys.path.insert(1, "../")

import analysis.processing.process_sa
import analysis.processing.process_ni
import analysis.processing.process_ot

import analysis.processing.process_s1
import analysis.processing.process_s2
import analysis.processing.process_s3

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['PROPAGATE_EXCEPTIONS'] = True

cam_images = {}

# where the robots reset to
robot_reset_positions = {
    "0": {
        "UAV1": [.522,.970],
        "UGV1": [.641,.970],
    },
    "1": {
        "UAV1": [.522,.970],
        "UAV2": [.641,.970],
        "UAV3": [.522,.980],
        "UAV4": [.641,.980],
    },
    "2": {
        "UAV1": [.522,.970],
        "UGV1": [.641,.970],
        "UAV2": [.522,.970],
        "UGV2": [.641,.970],
        "UAV3": [.522,.970],
        "UGV3": [.641,.970],
        "UAV4": [.522,.970],
        "UGV4": [.641,.970],
    },
    "3": {
        "UGV1": [.522,.970],
        "UGV2": [.641,.970],
        "UGV3": [.522,.980],
        "UGV4": [.641,.980],
    },
}

robot_positions = {}
robot_waypoints = {}
robot_publishers = {}

completions = {}

def robotPositionCallback(data):
    global robot_positions
    data = str(data.data).split(",")
    robot_positions[data[0]] = [float(data[1]), float(data[2])]
    #print("Updated robot position", data[0], robot_positions[data[0]])
    return


threading.Thread(target=lambda: rospy.init_node('dcistserver', disable_signals=True)).start()

# ROS callback function for the camera feeds, gets the image component and adds it to the right spot
def robotCameraCallback(data):
    data = str(data.data)
    vehicle = data[:4]
    image = data[7:-1] # b' ... '
    cam_images[vehicle] = image
    return


# ROS callback function for the next waypoint
def robotAtWaypointCallback(data):
    data = str(data.data).split(",")
    robot = data[0]
    waypoint = [round(float(data[1]), 3), round(float(data[2]), 3)]

    #print("Robot close to waypoint")

    if len(robot_waypoints[robot]) > 0 and [round(robot_waypoints[robot][0][0], 3), round(robot_waypoints[robot][0][1], 3)] == waypoint:        
        #print(">>>>Robot has reached a waypoint!")
        robot_waypoints[robot].pop(0)
        if len(robot_waypoints[robot]) > 0:  # if there are waypoints left, go to the next one
            robot_publishers[robot].publish(str(robot_waypoints[robot][0][0]) + "," + str(robot_waypoints[robot][0][1]))
        elif len(robot_waypoints[robot]) == 0:  # if there are no waypoint left, stop
            robot_publishers[robot].publish("stop")
    

def initROSSubscribers(stage):
    global robot_publishers
    global cam_images
    #print(">>>", robot_reset_positions[stage])
    if stage == "1" or stage == "3":
        for name in robot_reset_positions[stage]:
            rospy.Subscriber("/" + name + "/current_position", String, robotPositionCallback, queue_size=1)
            rospy.Subscriber("/" + name + "/current_image", String, robotCameraCallback, queue_size=1)
            rospy.Subscriber("/" + name + "/at_waypoint", String, robotAtWaypointCallback, queue_size=1)
            pub = rospy.Publisher("/" + name + "/set_position", String, queue_size=5)
            robot_publishers[name] = pub
            cam_images[name] = ""
            robot_waypoints[name] = []


@app.route("/", methods=["GET"])
def index():
    mission = str(request.args.get("mission"))
    return render_template("simenv/index.html", mission=mission)

@app.route("/stage", methods=["GET"])
def stage():
    global robot_positions
    mission = str(request.args.get("mission"))
    worker_id = request.args.get("workerId")
    stage = str(request.args.get("stage"))
    #print(">>>", mission, worker_id, stage, robot_reset_positions.keys())
    robot_positions = robot_reset_positions[stage]

    if stage == 2:
        robot_positions = {
            "UAV1": [.522,.970],
            "UGV1": [.641,.970],
            "UAV2": [.522,.970],
            "UGV2": [.641,.970],
            "UAV3": [.522,.970],
            "UGV3": [.641,.970],
            "UAV4": [.522,.970],
            "UGV4": [.641,.970],
        }

    # init the ROS subscribers and some other variables
    # if the stage is not stage 2
    if stage != 2:
        initROSSubscribers(stage)
    
    return render_template("simenv/stage.html", mission=mission, worker_id=worker_id, stage=stage, robot_positions=robot_positions)

@app.route("/portal", methods=["GET"])
def portal():
    mission = str(request.args.get("mission"))
    worker_id = str(request.args.get("workerId"))
    page_from = str(request.args.get("pageFrom"))
    success = str(request.args.get("success"))

    # if coming successfully from a page, mark it
    if page_from in ["0", "1", "2", "3"] and success == "1" and worker_id in completions:
        current_completions = completions[worker_id]
        current_completions = current_completions[:int(page_from)] + '1' + current_completions[int(page_from)+1:]
        completions[worker_id] = current_completions
        print("updated completions for", worker_id, "to", current_completions)

    # get the completions set
    completion_string = "0000"
    if worker_id in completions:
        completion_string = completions[worker_id]
        print("existing worker id, pulling completions", completion_string)
    elif worker_id != "":
        completions[worker_id] = "0000"
    else:
        worker_id = "none"

    # generate the selected stage if the training is complete
    next_stage = "0"
    if completion_string in ["1110", "1100", "1010"]:
        next_stage = 3  # this indicates we are now doing the robot mission, the mission variable will set the particular mission
    elif completion_string[0] == "1" and completion_string != "1111" and completion_string != "1011" and completion_string != "1101":
        next_stage = 1 # currently only selecting SA test (1) or the NC test (2)
    
    print("Next stage:", next_stage, completion_string, completion_string in ["1110", "1100", "1010"])

    completion_code = "complete the missions first!"
    if completion_string in ["1101", "1011", "1111"]:
        next_stage = -1
        completion_code = "1985636"

    return render_template("simenv/portal.html", mission=mission, worker_id=worker_id, completions=completion_string, next_stage=int(next_stage), completion_code=completion_code)

@app.route("/admin", methods=["GET"])
def admin():
    return render_template("admin/admin.html")

@app.route("/screen", methods=["GET"])
def screen():
    next_stage = str(request.args.get("nextStage"))
    return render_template("tutorial/screen.html", nextStage=next_stage)

@app.route("/tutorial", methods=["GET"])
def tutorial():
    mission = str(request.args.get("mission"))
    worker_id = request.args.get("workerId")
    next_stage = request.args.get("nextStage")
    test_stage = request.args.get("testStage")
    return render_template("tutorial/index.html", mission=mission, worker_id=worker_id, next_stage=next_stage, test_stage=test_stage)

@app.route("/test", methods=["GET"])
def testStage():
    global robot_positions
    mission = request.args.get("mission")
    worker_id = request.args.get("workerId")
    next_stage = request.args.get("nextStage")

    robot_positions = robot_reset_positions["0"]
    return render_template("simenv/stage.html", mission=mission, stage=0, next_stage=next_stage, worker_id=worker_id, robot_positions=str({}))

@app.route("/logging", methods=["POST"])
def log():
    data = json.loads(request.data.decode())
    # ignore if the worker ID is not specified
    if "worker-id" not in data:
        return ""
    log_string = "\n" + str(datetime.datetime.now().timestamp()) + "," + data["worker-id"] + "," + str(data)
    with open("./logs/" + data["worker-id"] + ".txt", 'a+') as f:
        f.write(log_string)
    print("[LOG]", log_string[1:])

    return ""

@app.route("/cams")
def cam():
    return jsonify(cam_images)

@app.route("/positions", methods=["GET"])
def position():
    return jsonify(robot_positions)

@app.route("/add-waypoint", methods=["GET"])
def addWaypoint():
    #print("adding wayyyypoints")
    robot = request.args.get("id")
    x = float(request.args.get("x"))
    y = float(request.args.get("y"))
    # if robot not in the waypoint system, make it
    if robot not in robot_waypoints:
        robot_waypoints[robot] = []
    # add the waypoint to the robot
    robot_waypoints[robot].append([x, y])
    # if this new waypoint was the only one in the system, start going to it
    if len(robot_waypoints[robot]) == 1:
        robot_publishers[robot].publish(str(x) + "," + str(y))
        print("published ", robot, str(x) + "," + str(y))

    return "success"

@app.route("/remove-waypoint", methods=["GET"])
def removeWaypoint():
    global robot_waypoints
    robot = request.args.get("id")
    if robot not in robot_waypoints:
        robot_waypoints[robot] = []

    if len(robot_waypoints[robot]) > 0:
        robot_waypoints[robot].pop()

    if len(robot_waypoints[robot]) == 0:
        robot_publishers[robot].publish("stop")

    return "success"

@app.route("/remove-all-waypoints", methods=["GET"])
def removeAllWaypoints():
    global robot_waypoints
    robot = request.args.get("id")
    robot_waypoints[robot] = []
    robot_publishers[robot].publish("stop")
    return "success"

# for when a robot reaches a waypoint, remove it from the list and let WeBots know
@app.route("/at-waypoint", methods=["GET"])
def atWaypoint():
    global robot_waypoints
    robot = request.args.get("id")
    robot_waypoints[robot].pop(0)
    if len(robot_waypoints[robot]) > 0:  # if there are waypoints left, go to the next one
        robot_publishers[robot].publish(str(robot_waypoints[robot][0][0]) + "," + str(robot_waypoints[robot][0][1]))
    elif len(robot_waypoints[robot]) == 0:  # if there are no waypoint left, stop
        robot_publishers[robot].publish("stop")
    return "success"


@app.route("/get-waypoints", methods=["GET"])
def getWaypoints():
    return jsonify(robot_waypoints)


# ROUTES FOR THE SURVEY TEST
@app.route("/survey", methods=["GET"])
def survey():
    worker_id = request.args.get("workerId")
    return render_template("survey/index.html", worker_id=worker_id)


# ROUTES FOR THE SITUATIONAL AWARENESS TEST
@app.route("/sa-test", methods=["GET"])
def saTest():
    worker_id = request.args.get("workerId")
    mission = request.args.get("mission")
    return render_template("sa-test/index.html", worker_id=worker_id, mission=mission)


# ROUTES FOR THE NETWORK CONNECTIVITY TEST
@app.route("/connect-data", methods=["POST"])
def networkConnectivityData():
    data = request.data.decode()
    log_string = "\n" + str(datetime.datetime.now().timestamp()) + "," + data.split(",")[0] + "," + str(data)
    with open("./logs/" + data.split(",")[0] + ".txt", 'a+') as f:
        f.write(log_string)

    print("[LOG]", log_string[1:])
    return "success"


@app.route("/connect", methods=["GET"])
def networkConnectivity():
    worker_id = request.args.get("workerId")
    mission = request.args.get("mission")
    return render_template("networks/connectivity.html", worker_id=worker_id, mission=mission)


@app.route("/ni-test", methods=["GET"])
def niTest():
    worker_id = request.args.get("workerId")
    mission = request.args.get("mission")
    return render_template("ni-test/index.html", worker_id=worker_id, mission=mission)


# ROUTES FOR THE OBJECT TRACKING TEST
@app.route("/ot-test")
def otTest():
    worker_id = request.args.get("workerId")
    mission = request.args.get("mission")
    return render_template("ot-test/index.html", worker_id=worker_id, mission=mission)


# ROUTE FOR DATA PROCESSING
@app.route("/process-data")
def processData():
    sa_data = analysis.processing.process_sa.get_sa_data("logs")
    ni_data = analysis.processing.process_ni.get_ni_data("logs")
    ot_data = analysis.processing.process_ot.get_ot_data("logs")
    s1_data = analysis.processing.process_s1.get_s1_data("logs")
    s2_data = analysis.processing.process_s2.get_s2_data("logs")
    s3_data = analysis.processing.process_s3.get_s3_data("logs")

    total = {}
    
    for p in {**sa_data, **ni_data, **ot_data, **s1_data, **s2_data, **s3_data}:
        if "email" in p or p == "0":
            continue
        if p not in total:
            total[p] = {}
        total[p]["sa"] = sa_data[p] if p in sa_data else -1
        total[p]["ni"] = ni_data[p] if p in ni_data else -1
        total[p]["ot"] = ot_data[p] if p in ot_data else -1
        total[p]["s1"] = s1_data[p] if p in s1_data else -1
        total[p]["s2"] = s2_data[p] if p in s2_data else -1
        total[p]["s3"] = s3_data[p] if p in s3_data else -1
    

    
    
    return jsonify(total)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
