from flask import Flask, render_template, request, make_response

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import threading
import cv2
import numpy as np

app = Flask(__name__)
bridge = CvBridge()

cam_images = {
    1: "",
    2: "",
    3: "",
    4: "",
}

robot_positions = {
    1: [.1,.1],
    2: [.2,.2],
    3: [.3,.3],
    4: [.4,.4],
}


def cam1_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[cam] = b64
    return

def cam2_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        retval, buffer = cv2.imencode('.png', cv_image)
        b64 = base64.b64encode(buffer)
        cam_images[cam] = b64
    except Exception as e:
        print("EXCEPT", e.text())
    return

def cam3_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[cam] = b64
    return

def cam4_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[cam] = b64
    return

threading.Thread(target=lambda: rospy.init_node('dcistserver', disable_signals=True)).start()
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam1_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam2_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam3_callback)
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam4_callback)

@app.route("/")
def index():
    return render_template("main.html")

@app.route("/appy")
def appy():
    return render_template("main.html")

@app.route("/cam", methods=["GET"])
def cam():
    try:
        cam = int(request.args.get("id"))
        res = cam_images[cam]
    except Exception as e:
        print("EXCEPT", e.text)
    return res

@app.route("/position", methods=["GET"])
def position():
    robot = int(request.args.get("id"))
    return robot_positions[robot]


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
