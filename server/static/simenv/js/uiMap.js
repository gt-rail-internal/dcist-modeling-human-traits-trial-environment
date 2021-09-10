class UIMap {

    constructor(canvasId) {
        this.mapCanvas = document.getElementById(canvasId);
        this.mapContext = this.mapCanvas.getContext('2d');

        // make arrays of the UI Objects
        this.uiObjects = [];
        this.uiObstacles = [];

        // variable for the selected game object
        this.selectedObject = null;
        this.selectorImage = new Image();
        this.selectorImage.src = "/static/simenv/img/selector.png"

        // set the image 
        // adapted from https://riptutorial.com/html5-canvas/example/19169
        this.mapImage = new Image();
        this.mapImage.src = "/static/simenv/img/map.png"

        this.mapScale = 1;  // the map's scale, so the image fits the canvas

        // variable for the current stage
        this.stage = 0;
        this.stageComplete = false;
        this.stageVictory = false;
        this.nextStage = 0;

        // variables for the REPLAY
        this.replay = false;
        this.maxConnected = 0;

        // variables for stage 3
        this.announcedStuck = false;

        // variable for the end function
        this.endCheck = null;

        // variable for the cameras, names of the vehicles
        this.camNames = [];

        // variable for distance traveled by each robot
        this.distanceTraveled = [0, 0, 0, 0, 0, 0, 0, 0];

        // variable for the robot locations and orientations
        this.robotLocations = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]];

        // variable for the cache states
        // Stage 1: 0=unmarked, 1=marked
        // Stage 2: 0=unconnected, 1=connected
        // Stage 3: 0=uncollected, 1X=connected on robot_id X, 2=returned
        this.cacheStates = [0, 0, 0, 0, 0]

        // variable for connected cache
        this.cacheDisconnected = true;

        // variable for the training round
        this.training = false;

        // variable for the worker ID
        this.workerID = "invalid"
        this.interacted = false;  // turns true once the user interacts with the map

        // variables for drawn items
        this.adHocRange = 100;
        this.displayAdHocRanges = true;
        this.adHocLock = true;

        // variables for functions run often
        this.checkButtons = undefined;

        // variables for stage 1
        this.knownCaches = 0;

        // variables for stage 3
        this.returnedCaches = 0;
        this.mainBase = undefined;

        this.goalLocations = [];
        this.displayGoalLocations = true;

        this.goalHelpers = [];
        this.displayGoalHelpers = true;

        // variables for a red canvas arrow, if one exists
        this.redWaypointTime = 0;
        this.redWaypointLocation = [];

        // flag for whether to use networks (false only on Stage 2)
        this.networked = true;

        // when the image loads, scale it
        var obj = this;
        this.mapImage.onload = function() {
            obj.mapScale = Math.min(obj.mapCanvas.width / obj.mapImage.width, obj.mapCanvas.height / obj.mapImage.height);
        }
    }

    // draws the map and UI objects
    draw() {
        // clear the map
        this.mapContext.clearRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        // draw the map
        this.mapContext.drawImage(this.mapImage, 0, 0, this.mapImage.width * this.mapScale, this.mapImage.height * this.mapScale);

        // draw the UI objects
        for (var i in this.uiObjects) {
            this.uiObjects[i].draw();
            // draw the vehicle-specific attributes
            if (this.uiObjects[i].constructor.name == "Vehicle") {
                // draw grey waypoints for non-selected object, black for selected object
                var color = this.selectedObject == this.uiObjects[i] ? "black" : "grey"
                this.uiObjects[i].drawWaypoints(color);
            }

            if (this.uiObjects[i].constructor.name == "Vehicle" || this.uiObjects[i].constructor.name == "Router") {
                // draw the adhoc range (will only draw if applicable)
                if (this.displayAdHocRanges) {
                    this.uiObjects[i].drawAdHoc();   
                }
            }

            // draw the base-specific attributes
            if (this.uiObjects[i].constructor.name == "Base") {
                // draw the adhoc range (will only draw if applicable stage)
                if (this.displayAdHocRanges) {
                    this.uiObjects[i].drawAdHoc();
                }
            }
        }

        // if a vehicle is selected...
        if (this.selectedObject) {
            // draw the selection circle
            drawCircle(this.mapContext, this.selectedObject.x, this.selectedObject.y, this.selectedObject.scale, this.selectedObject.color);
        }

    }
}


