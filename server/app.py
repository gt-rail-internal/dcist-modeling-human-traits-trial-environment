from flask import Flask, render_template, request, make_response

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import threading
import cv2

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
    cam_images[1] = base64.b64encode(buffer)
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
    
    # return the image
    # adapted from https://stackoverflow.com/questions/3715493/
    try:
        encoded_string = base64.b64encode(cam_images[cam])
    except Exception as e:
        encoded_string = "fail"
        print("FAIL", e.text())
    print("got cam image", cam, len(encoded_string))
    
    return encoded_string


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)