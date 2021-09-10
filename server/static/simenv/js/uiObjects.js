class MapUIObject {
    constructor() {
        this.canvas = document.getElementById("uimap-canvas");
        this.context = this.canvas.getContext('2d');

        this.index = -1;
        
        this.x = 0;
        this.y = 0;

        this.scale = 25;

        this.image = new Image();

        this.hide = false;  // if true, does not draw the object

        this.type = "";

        this.name = "";
        this.nameAttention = false;  // give the name attention if needed (! name !)

        this.carryingCache = false;
        this.carryingCacheId = -1; // ID of the cache being carried

        this.color = "";

        this.displayDot = false;  // displays the colored dot in the center of the object
        this.displayName = false;   // displays the name of the object

        this.selectable = false;  // allows this object to be selected
    }

    // draws the image on the map canvas
    draw() {
        // if hiding the object, return
        if (this.hide) {
            return;
        }

        // draw the image if one exists
        if (this.image && this.scale && this.constructor.name != "Vehicle") {
            this.context.drawImage(this.image, this.x-this.scale/2, this.y-this.scale/2, this.scale, this.scale);
        }
        // for vehicles, draw a rotated image
        else if (this.image && this.scale && this.constructor.name == "Vehicle") {
            drawRotated(this.context, this.image, this.x, this.y, this.scale, -Math.atan2(this.y - this.oldY, this.oldX - this.x) + Math.PI/2);
        }

        // for a base, draw dropoff circle around it if on Stage 3
        if (this.constructor.name == "Base" && uiMap.stage == 3) {
            drawCircle(this.context, this.x, this.y, .05 * uiMap.mapCanvas.width, "lightblue");
        }

        // set the fill color
        this.context.fillStyle = this.color;

        // if a cache, set color depending on whether it is connected
        if (this.type == "cache" && this.connected) {
            this.context.fillStyle = "yellow"
        }

        // draw the name
        // figure out what the name should be for vehicles
        if (this.displayName) {
            let objectName = uiMap.stageComplete ? "Complete" : this.type == "ugv" ? "Ground" : this.type == "uav" ? "Aerial" : this.type == "cache" ? this.connected ? "Connected" : "Cache" : this.name;
            if (this.carryingCache) {
                objectName = "📦 " + objectName;
            }
            this.context.font = this.nameAttention ? "bold 20px Arial" : "20px Arial";
            this.context.textAlign = "center";
            this.context.fillText(objectName, this.x, this.y - this.scale * .75);
        }

        // draw the color dot
        if (this.displayDot) {
            this.context.beginPath();
            this.context.arc(this.x, this.y, this.scale / 5, 0, 2 * Math.PI, false);
            this.context.fill();
        }

        // if has a fill radius, fill it
        if (this.fillRadius) {
            this.context.fillStyle = "rgba(250, 250, 250, .5)";
            this.context.beginPath();
            this.context.arc(this.x, this.y, this.fillRadius * this.canvas.width, 0, 2 * Math.PI, false);
            this.context.fill();
        }

        // reset the fill color
        this.context.fillStyle = "black";
        return;
    }

    // get the location
    getLoc() {
        return [this.x, this.y];
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
        this.adHocLocked = false;

        // initialize the vehicle travel speed (for the Stage 2 simulated motion)
        this.speed = 10;  // pixels / sec

        this.oldX = 0;
        this.oldY = 0;

        this.stuckX = -1;
        this.stuckY = -1;
        this.stuckTimer = 0;

        // set some vehicle attributes by the type
        if (this.type == "uav") {
            this.image.src = "/static/simenv/img/uav.png";   
            this.adHocRadius = 75;
            this.speed = 10;
        }

        if (this.type == "ugv") {
            this.image.src = "/static/simenv/img/ugv.png";
            this.adHocRadius = 100;
            this.speed = 5;
        }

        this.scale = 25;  // set the object scale to 25px

        this.displayDot = true;
        this.displayName = true;

        // initialize the vehicle waypoints
        this.waypoints = new Array();

        // red waypoint, if applicable (invalid waypoint item)
        this.redWaypointTime = 0;
        this.redWaypointLocation = [];

        // allow the object to be selected
        this.selectable = true;

    }

    // update the vehicle position
    updatePosition(x, y) {
        this.x = x;
        this.y = y;
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

        // draw a red waypoint if applicable
        let time = new Date().getTime() / 1000;
        if (time - this.redWaypointTime < 0.5) {
            this.context.strokeStyle = "red";
            let lastWaypoint = this.getLastWaypoint();
            canvas_arrow(this.context, lastWaypoint[0] * this.canvas.width, lastWaypoint[1] * this.canvas.height, this.redWaypointLocation[0], this.redWaypointLocation[1]);
            this.context.lineWidth = 1.5;
            this.context.stroke();
        }
    }

    // utility function to get the pixel coordinates of the last waypoint
    getLastWaypoint() {
        if (this.waypoints.length > 0) {
            return [this.waypoints[this.waypoints.length - 1][0], this.waypoints[this.waypoints.length - 1][1]];
        }
        else {
            return [this.x / this.canvas.width, this.y / this.canvas.height];
        }
    }

    drawAdHoc() {
        drawDashedCircle(this.context, this.x, this.y, this.adHocRadius, this.nameAttention ? "red" : "black");
    }

}


// Cache objects are the packages
class Cache extends MapUIObject {
    constructor() {
        super();

        this.type = "cache";

        this.name = "Cache";
        this.color = "black";

        this.index = -1;

        this.image.src = "/static/simenv/img/cache.png";
        this.scale = 25;

        this.displayName = true;
        this.displayDot = false;

        this.adHocRadius = 0;

        // flag for if the cache is connected
        this.connected = false;
    }
}

// Cache Area objects are locations where a cache could be (for Round 1). It is just a grey circle
class CacheArea extends MapUIObject {
    constructor() {
        super();

        this.type = "cache area";
        this.name = "";
        this.color = "lightgrey";

        this.fillRadius = .2;

        this.cacheX = 0
        this.cacheY = 0
    }
}

// Router objects are invisible non-selectable objects that have an adhoc range
class Router extends MapUIObject {
    constructor() {
        super();

        this.type = "router";

        // initialize the ad hoc radius
        this.adHocRadius = 100;
        this.adHocLocked = false;
    }

    // drawing function to draw the adhoc circle
    drawAdHoc() {
        drawDashedCircle(this.context, this.x, this.y, this.adHocRadius, this.nameAttention ? "red" : "black");
    }
}

// Base objects are where the robots come/return to
class Base extends MapUIObject {
    constructor() {
        super();

        this.type = "base";

        this.adHocRadius = 100;

        this.image.src = "/static/simenv/img/base.png";
        this.scale = 40;

        this.name = "Base";
        this.color = "black";

        this.displayName = true;
        this.displayDot = false;
    }

    drawAdHoc() {
        drawDashedCircle(this.context, this.x, this.y, this.adHocRadius, "black");
    }
}

class MapUIObstacle {
    constructor() {
        this.shape = [];  // array of canvas points in canvas % form
    }

    // checks if a waypoint goes through/inside the shape
    checkWaypointIntersects(waypoint) {
        let waypoint_start = waypoint[0];
        let waypoint_end = waypoint[1];

        // loop through all segments in the shape
        for (i in this.shape) {
            i = parseInt(i);

            // check if going out of bounds
            if (i + 1 > this.shape.length - 1) {
                break;
            }

            let segment_start = this.shape[i];
            let segment_end = this.shape[i+1];

            // check if intersect, return true
            if (this.isIntersecting(waypoint_start, waypoint_end, segment_start, segment_end)) {
                return true;
            }
        }

        // check last segment (first/last indices)
        if (this.isIntersecting(waypoint_start, waypoint_end, this.shape[this.shape.length-1], this.shape[0])) {
            return true;
        }

        // if none intersected, return false
        return false;
    }

    // utility function to see if two lines intersect
    isIntersecting(p1, p2, p3, p4) {
        function CCW(p1, p2, p3) {
            return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0]);
        }
        return (CCW(p1, p3, p4) != CCW(p2, p3, p4)) && (CCW(p1, p2, p3) != CCW(p1, p2, p4));
    }
}
