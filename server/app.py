from flask import Flask, render_template, request, make_response

import rospy
from sensor_msgs.msg import CompressedImage
import base64

app = Flask(__name__)

cam1_image = ""
cam2_image = ""
cam3_image = ""
cam4_image = ""


def cam1_callback(data):
    print("got image!")
    return
        

# create a ros node and the subscribers
def rosnode():
    rospy.init_node('dcistserver', anonymous=True)
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, cam1_callback)
 
    rospy.spin()


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
    filename = "static/img/cams/cam" + str(cam) + ".png"
    with open(filename, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read())
    
    return encoded_string


rosnode()
app.run(host="0.0.0.0", port=5000)
