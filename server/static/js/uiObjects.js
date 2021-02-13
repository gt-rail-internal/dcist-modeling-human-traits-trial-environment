class MapUIObject {
    constructor() {
        this.canvas = document.getElementById("uimap-canvas");
        this.context = this.canvas.getContext('2d');

        this.index = -1;
        
        this.x = 0;
        this.y = 0;

        this.scale = 25;

        this.image = new Image();

        this.name = "";
        this.color = "";
    }

    // draws the image on the map canvas
    draw() {
        // draw the UAV
        this.context.drawImage(this.image, this.x-this.scale/2, this.y-this.scale/2, this.scale, this.scale);

        // set the fill color
        this.context.fillStyle = this.color;

        // draw the name
        this.context.font = "20px Arial";
        this.context.textBaseline = "middle";
        this.context.fillText(this.name, this.x - this.scale, this.y - this.scale);

        // draw the color
        this.context.beginPath();
        this.context.arc(this.x, this.y, this.scale / 5, 0, 2 * Math.PI, false);
        this.context.fill();

        // reset the fill color
        this.context.fillStyle = "black";
    }
}


// Vehicle objects are UAVs and UGVs
class Vehicle extends MapUIObject {
    constructor(type) {
        // call the superclass
        super();

        // set the UI object type
        this.type = type;

        // initialize the ad hoc radius
        this.adHocRadius = 100;

        // set some vehicle attributes by the type
        if (this.type == "uav") {
            this.image.src = "/static/img/uav.png";   
            this.adHocRadius = 75;
        }

        if (this.type == "ugv") {
            this.image.src = "/static/img/ugv.png";
            this.adHocRadius = 100;
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
    }

    drawWaypoints(color) {
        let fromX = 0;
        let fromY = 0;
        let toX = 0;
        let toY = 0;

        // draw the waypoint arrows
        this.context.beginPath();
        if (this.waypoints.length > 0) {
            fromX = this.x;
            fromY = this.y;
        }
        for (var i in this.waypoints) {
            toX = this.waypoints[i][0] * this.canvas.width;
            toY = this.waypoints[i][1] * this.canvas.height;

            canvas_arrow(this.context, fromX, fromY, toX, toY);

            fromX = toX;
            fromY = toY;
        }
        this.context.strokeStyle = color;
        this.context.lineWidth = 1.5;
        this.context.stroke();
        this.context.strokeStyle = color;
    }

    drawAdHoc() {
        drawDashedCircle(this.context, this.x, this.y, this.adHocRadius);
    }

}


// Cache objects are the packages
class Cache extends MapUIObject {
    constructor() {
        super();

        this.type = "cache";

        this.image.src = "/static/img/cache.png";
        this.scale = 75;
    }
}