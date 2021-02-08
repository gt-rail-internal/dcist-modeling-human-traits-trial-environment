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

def cam1_callback(data):
    print("updated image cam1")
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    retval, buffer = cv2.imencode('.png', cv_image)
    b64 = base64.b64encode(buffer)
    cam_images[1] = b64
    #cv_image = cv2.imdecode(np.array(b64), cv2.IMREAD_COLOR)
    #cv2.imwrite("imageee.png", cv_image)
    return

threading.Thread(target=lambda: rospy.init_node('dcistserver', disable_signals=True)).start()
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam1_callback)

@app.route("/")
def index():
    return render_template("main.html")

@app.route("/appy")
def appy():
    return render_template("main.html")

@app.route("/cam", methods=["GET"])
def cam():
    cam = request.args.get("id")
    # check if the cam id is a number
    if cam.isnumeric():
        cam = int(cam)
    else:
        return "ID must be an integer"
    
    # check if the cam id is valid
    if cam > 4 or cam < 1:
        return "ID must be between 1 and 4 (inclusive)"
    
    print("got cam image", cam)
    
    return cam_images[cam]


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)