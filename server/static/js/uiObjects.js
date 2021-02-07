class MapUIObject {
    constructor() {
        this.canvas = document.getElementById("uimap-canvas");
        this.context = this.canvas.getContext('2d');
        
        this.x = 0;
        this.y = 0;

        this.scale = 25;

        this.image = new Image();
    }

    // draws the image on the map canvas
    draw() {
        this.context.drawImage(this.image, this.x-this.scale/2, this.y-this.scale/2, this.scale, this.scale);
    }
}


// Vehicle objects are UAVs and UGVs
class Vehicle extends MapUIObject {
    constructor(type) {
        // call the superclass
        super();

        // set the UI object type
        this.type = type;

        // set the vehicle image by the type
        if (this.type == "uav") {
            this.image.src = "/static/img/uav.png";   
        }

        if (this.type == "ugv") {
            this.image.src = "/static/img/ugv.png"
        }

        this.scale = 25;  // set the object scale to 25px

        // initialize the vehicle waypoints
        this.waypoints = new Array();
    }

    // update the vehicle position
    updatePosition(x, y) {
        this.x = x;
        this.y = y;
    }

    // set up a subscriber to a Point ROS message for the vehicle position
    initPositionListener(topic) {
        var positionTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/' + topic,
            messageType: 'geometry_msgs/Point', 
        });

        positionTopic.subscribe((message) => {
            console.log("ROS position update for " + positionTopic.name + ": (" + message.x + ", " + message.y + ")");
            this.x = this.canvas.width * message.x;
            this.y = this.canvas.height * message.y;
        })
    }

}