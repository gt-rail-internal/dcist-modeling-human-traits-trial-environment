from flask import Flask, render_template, request
import rospy

import base64

app = Flask(__name__)

@app.route("/")
def index():
    return "Server is online"

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
    filename = "img/cam" + str(cam) + ".png"
    with open(filename, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read())

    return encoded_string

app.run(host="0.0.0.0", port=5000)