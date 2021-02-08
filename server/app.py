from flask import Flask, render_template, request, make_response

import rospy
from sensor_msgs.msg import Image
import base64
import threading

app = Flask(__name__)

cam_images = {
    1: "",
    2: "",
    3: "",
    4: "",
}

def cam1_callback(data):
    cam_images[1] = data.data
    return

threading.Thread(target=lambda: rospy.init_node('dcistserver', disable_signals=True)).start()
rospy.Subscriber("/raspicam_node/image/compressed", Image, cam1_callback)

@app.route("/")
def index():
    return render_template("index.html")

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
    encoded_string = base64.b64encode(cam_images[cam])
    
    return encoded_string




app.run(host="0.0.0.0", port=5000)