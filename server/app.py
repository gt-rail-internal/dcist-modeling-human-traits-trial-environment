from flask import Flask, render_template, request, make_response, jsonify

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import threading
import cv2
import numpy as np
import json
import datetime
import random

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['PROPAGATE_EXCEPTIONS'] = True
bridge = CvBridge()

cam_images = {
    1: "",
    2: "",
    3: "",
    4: "",
}

robot_positions = {
    "UAV 1": [.522,.970],
    "UGV 1": [.641,.970],
    "UAV 2": [.522,.970],
    "UGV 2": [.641,.970],
    "UAV 3": [.522,.970],
    "UGV 3": [.641,.970],
    "UAV 4": [.522,.970],
    "UGV 4": [.641,.970],
}

robot_waypoints = {
    "UAV 1": [],
    "UGV 1": [],
    "UAV 2": [],
    "UGV 2": [],
    "UAV 3": [],
    "UGV 3": [],
    "UAV 4": [],
    "UGV 4": [],
}

completions = {}


def cam1_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[1] = b64
    return

def cam2_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[2] = b64
    return

def cam3_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[3] = b64
    return

def cam4_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[4] = b64
    return

def robot1_callback(data):
    robot_positions[1] = data

def robot2_callback(data):
    robot_positions[2] = data

def robot3_callback(data):
    robot_positions[3] = data

def robot4_callback(data):
    robot_positions[4] = data


threading.Thread(target=lambda: rospy.init_node('dcistserver', disable_signals=True)).start()
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam1_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam2_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam3_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam4_callback)

rospy.Subscriber("/robot1/pos", Image, cam1_callback)
rospy.Subscriber("/robot1/pos", Image, cam2_callback)
rospy.Subscriber("/robot1/pos", Image, cam3_callback)
rospy.Subscriber("/robot1/pos", Image, cam4_callback)

@app.route("/")
def index():
    return render_template("simenv/index.html")

@app.route("/stage", methods=["GET"])
def stage():
    worker_id = request.args.get("workerId")
    stage = request.args.get("stage")
    return render_template("simenv/stage.html", worker_id=worker_id, stage=stage)

@app.route("/portal", methods=["GET"])
def portal():
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
    if completion_string == "1110":
        next_stage = "3"
    elif completion_string[0] == "1" and completion_string != "1111":
        potentials = []
        for i in range(len(completion_string) - 1):  # randomize the pretests
            if completion_string[i] == "0":
                potentials.append(str(i))
        next_stage = potentials[random.randint(0, len(potentials) - 1)]
    
    completion_code = "complete the missions first!"
    if completion_string == "1111":
        next_stage = -1
        completion_code = "415626404"

    return render_template("simenv/portal.html", worker_id=worker_id, completions=completion_string, next_stage=int(next_stage), completion_code=completion_code)

@app.route("/tutorial", methods=["GET"])
def tutorial():
    worker_id = request.args.get("workerId")
    next_stage = request.args.get("nextStage")
    return render_template("tutorial/index.html", worker_id=worker_id, next_stage=next_stage)



@app.route("/test", methods=["GET"])
def testStage():
    worker_id = request.args.get("workerId")
    return render_template("simenv/stage.html", stage=0, worker_id=worker_id)

@app.route("/logging", methods=["POST"])
def log():
    data = json.loads(request.data.decode())
    log_string = "\n" + str(datetime.datetime.now().timestamp()) + "," + data["worker-id"] + "," + str(data)
    with open("./logs/" + data["worker-id"] + ".txt", 'a+') as f:
        f.write(log_string)
    print("[LOG]", log_string[1:])

    return ""

@app.route("/cams")
def cam():
    res = [cam_images[1], cam_images[2], cam_images[3], cam_images[4]]
    return jsonify(res)

@app.route("/positions", methods=["GET"])
def position():
    return jsonify(robot_positions)

@app.route("/add-waypoint", methods=["GET"])
def addWaypoint():
    #print("adding wayyyypoints")
    robot = request.args.get("id")
    x = float(request.args.get("x"))
    y = float(request.args.get("y"))
    #print(">>>>", robot, x, y, robot_waypoints[robot])
    robot_waypoints[robot].append([x, y])
    return "success"

@app.route("/remove-waypoint", methods=["GET"])
def removeWaypoint():
    robot = request.args.get("id")
    robot_waypoints[robot].pop()
    return "success"

@app.route("/remove-all-waypoints", methods=["GET"])
def removeAllWaypoints():
    robot = request.args.get("id")
    robot_waypoints[robot] = []
    return "success"

@app.route("/get-waypoints", methods=["GET"])
def getWaypoints():
    #print("waypoints", response)
    return jsonify(robot_waypoints)


# ROUTES FOR THE SITUATIONAL AWARENESS TEST
@app.route("/sa-test", methods=["GET"])
def saTest():
    worker_id = request.args.get("workerId")
    return render_template("sa-test/index.html", worker_id=worker_id)


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
    return render_template("networks/connectivity.html", worker_id=worker_id)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
