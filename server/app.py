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

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
bridge = CvBridge()

cam_images = {
    1: "",
    2: "",
    3: "",
    4: "",
}

robot_positions = {
    "UAV 1": [.40,.1],
    "UGV 1": [.40,.15],
    "UAV 2": [.5,.1],
    "UGV 2": [.5,.15],
    "UAV 3": [.65,.1],
    "UGV 3": [.65,.15],
    "UAV 4": [.75,.1],
    "UGV 4": [.75,.15],
}

robot_waypoints = {
    1: [],
    2: [],
    3: [],
    4: [],
}


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
    return "Use the /app route!"

@app.route("/app", methods=["GET"])
def appy():
    worker_id = request.args.get("workerId")
    return render_template("main.html", worker_id=worker_id)

@app.route("/logging", methods=["POST"])
def log():
    data = json.loads(request.data.decode())
    log_string = "\n" + str(datetime.datetime.now().timestamp()) + "," + data["worker-id"] + "," + str(data)
    with open("./logs/" + data["worker-id"] + ".txt", 'a+') as f:
        f.write(log_string)
        
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
    robot = int(request.args.get("id"))
    x = float(request.args.get("x"))
    y = float(request.args.get("y"))
    #print(">>>>", robot, x, y, robot_waypoints[robot])
    robot_waypoints[robot].append([x, y])
    return "success"

@app.route("/remove-waypoint", methods=["GET"])
def removeWaypoint():
    robot = request.args.get("id")
    robot_waypoints[int(robot)].pop()
    return "success"

@app.route("/get-waypoints", methods=["GET"])
def getWaypoints():
    response = [robot_waypoints[1], robot_waypoints[2], robot_waypoints[3], robot_waypoints[4]]
    #print("waypoints", response)
    return jsonify(response)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
