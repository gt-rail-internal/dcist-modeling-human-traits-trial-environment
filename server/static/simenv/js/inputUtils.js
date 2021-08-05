// handles the canvas mouse events
function canvasMouseHandler(event) {
    // if the mouse click is outside the range of the canvas, ignore it
    if (event.originalTarget != document.getElementById("uimap-canvas") && event.target != document.getElementById("uimap-canvas")) {
        return;
    }

    // if the stage is complete, don't do anything
    if (uiMap.stageComplete) {
        return;
    }

    if (!uiMap.interacted) {
        uiMap.interacted = true;
        trainingDisconnectRobot = 0;
    }

    var canvasX = event.clientX - (event.originalTarget ? event.originalTarget.offsetLeft : event.target.offsetLeft) + window.scrollX;
    var canvasY = event.clientY - (event.originalTarget ? event.originalTarget.offsetTop : event.target.offsetTop) + window.scrollY;
    var canvasClick = [canvasX, canvasY];

    console.log("click", canvasX / uiMap.mapCanvas.width, canvasY / uiMap.mapCanvas.height)

    clickedVehicle = false;

    // check if any vehicles were clicked
    for (i in uiMap.uiObjects) {
        if (uiMap.uiObjects[i].selectable && distance(canvasClick, [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < uiMap.uiObjects[i].scale * .86) {
            console.log("Selected vehicle");
            uiMap.selectedObject = uiMap.uiObjects[i];
            clickedVehicle = true;
            clickedVehicleIndex = i;
            log({action: "select-vehicle", target: uiMap.selectedObject.name});

            // set the stage to started
            if (!stageStarted) {
                stageStarted = true;
                startTime = new Date().getTime() / 1000;
            }

            // if on the training stage, increment that count
            if (uiMap.stage == 0) {
                trainingSelectRobot += 1;
                checkTraining();
            }
            break;
        }        
    }

    // if no vehicles were clicked, add a waypoint if applicable
    if (!clickedVehicle && uiMap.selectedObject != null && event.button == 0) {
        posX = canvasX / uiMap.mapCanvas.width;
        posY = canvasY / uiMap.mapCanvas.height;

        // check if waypoint is valid, if on stage 2
        if ((uiMap.stage == 2 || uiMap.stage == 0 ) && uiMap.selectedObject.type == "ugv") {
            priorWaypoint = uiMap.selectedObject.getLastWaypoint();

            // if the waypoint is not valid, turn it red and delete it after half a second
            if (!checkValidWaypoint(priorWaypoint, [posX, posY])) {
                uiMap.selectedObject.redWaypointTime = new Date().getTime() / 1000;
                uiMap.selectedObject.redWaypointLocation = [canvasX, canvasY];
                log({action: "add-invalid-waypoint", target: uiMap.selectedObject.name, location: "[" + posX + "," + posY + "]"});
                console.log("collision found")
                return;
            }
        }

        // if on stage 3 and robot is disconnected (name attention), don't move
        if (uiMap.stage == 3 && uiMap.selectedObject.nameAttention) {
            return;
        }


        uiMap.selectedObject.waypoints.push([posX, posY]);

        if (uiMap.networked) {
            fetch("add-waypoint?id=" + uiMap.selectedObject.name + "&x=" + posX + "&y=" + posY);
            console.log("added waypoint to server")
        }

        log({action: "add-valid-waypoint", target: uiMap.selectedObject.name, location: "[" + posX + "," + posY + "]"});
        console.log("Added waypoint");

        // update the training counter
        if (uiMap.stage == 0) {
            trainingAddWaypoints += 1;
            checkTraining();
        }
    }
}

function canvasKeypressHandler(event) {
    //console.log(event)
    // if an space key and an object is selected, remove the last waypoint
    if (event.key == "r" && uiMap.selectedObject != null) {
        // if training and the waypoints are now zero, update that training counter
        if (uiMap.stage == 0 && uiMap.selectedObject.waypoints.length == 1) {
            trainingStopRobot += 1;
            trainingRemoveWaypoints += 1;
            checkTraining();
        }

        // if training and not stopping a robot, just increase the "remove waypoints" training counter
        else if (uiMap.stage == 0 && uiMap.selectedObject.waypoints.length > 1) {
            trainingRemoveWaypoints += 1;
            checkTraining();
        }


        uiMap.selectedObject.waypoints.pop();
        if (uiMap.stage != 2) {
            fetch("remove-waypoint?id=" + uiMap.selectedObject.name);
        }
        log({action: "remove-waypoint", target: uiMap.selectedObject.name});
        console.log("Removed waypoint");
        
    }

    // if a enter key and an object is selected, remove the last waypoint
    if (event.key == "q" && uiMap.selectedObject != null) {
        log({action: "deselect-vehicle", target: uiMap.selectedObject.name});
        console.log("Deselected vehicle");
        uiMap.selectedObject = null;

        // if training stage, record the deselect
        if (uiMap.stage == 0) {
            trainingDeselectRobot += 1;
            checkTraining();
        }
    }
}