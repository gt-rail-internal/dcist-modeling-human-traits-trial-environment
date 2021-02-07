class UIMap {

    constructor(canvasId) {
        this.mapCanvas = document.getElementById(canvasId);
        this.mapContext = this.mapCanvas.getContext('2d');

        // make arrays of the UI Objects
        this.uiObjects = [];

        // variable for the selected game object
        this.selectedObject = null;
        this.selectorImage = new Image();
        this.selectorImage.src = "/static/img/selector.png"

        // set the image 
        // adapted from https://riptutorial.com/html5-canvas/example/19169
        this.mapImage = new Image();
        this.mapImage.src = "/static/img/map.png"

        this.mapScale = 1;  // the map's scale, so the image fits the canvas

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
        }

        // if a vehicle is selected...
        if (this.selectedObject) {
            // draw the selection circle
            this.mapContext.drawImage(this.selectorImage, this.selectedObject.x - this.selectedObject.scale, this.selectedObject.y - this.selectedObject.scale, this.selectedObject.scale * 2, this.selectedObject.scale * 2);
        
            let fromX = 0;
            let fromY = 0;
            let toX = 0;
            let toY = 0;

            // draw the waypoint arrows
            this.mapContext.beginPath();
            if (this.selectedObject.waypoints.length > 0) {
                fromX = this.selectedObject.x;
                fromY = this.selectedObject.y;
            }
            for (var i in this.selectedObject.waypoints) {
                toX = this.selectedObject.waypoints[i][0] * this.mapCanvas.width;
                toY = this.selectedObject.waypoints[i][1] * this.mapCanvas.height;

                canvas_arrow(this.mapContext, fromX, fromY, toX, toY);

                fromX = toX;
                fromY = toY;
            }
            this.mapContext.stroke();
        }

    }

    distance(a, b) {
        return Math.sqrt(Math.pow((a[0] - b[0]), 2) + Math.pow((a[1] - b[1]), 2));
    }
}
