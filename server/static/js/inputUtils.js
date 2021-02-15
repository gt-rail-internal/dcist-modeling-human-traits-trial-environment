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

    var canvasX = event.clientX - (event.originalTarget ? event.originalTarget.offsetLeft : event.target.offsetLeft);
    var canvasY = event.clientY - (event.originalTarget ? event.originalTarget.offsetTop : event.target.offsetTop);
    var canvasClick = [canvasX, canvasY];

    clickedVehicle = false;

    // check if any vehicles were clicked
    for (i in uiMap.uiObjects) {
        if (uiMap.uiObjects[i].selectable && distance(canvasClick, [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < uiMap.uiObjects[i].scale * .86) {
            console.log("Selected vehicle");
            uiMap.selectedObject = uiMap.uiObjects[i];
            clickedVehicle = true;
            clickedVehicleIndex = i;
            log({action: "select-vehicle", target: uiMap.selectedObject.name});
            break;
        }        
    }

    // if no vehicles were clicked, add a waypoint if applicable
    if (!clickedVehicle && uiMap.selectedObject != null && event.button == 0) {
        posX = canvasX / uiMap.mapCanvas.width;
        posY = canvasY / uiMap.mapCanvas.height;

        // check if waypoint is valid, if on stage 2
        if (uiMap.stage == 2 && uiMap.selectedObject.type == "ugv") {
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

        uiMap.selectedObject.waypoints.push([posX, posY]);

        if (uiMap.stage != 2) {
            fetch("add-waypoint?id=" + uiMap.selectedObject.index + "&x=" + posX + "&y=" + posY);
        }

        log({action: "add-valid-waypoint", target: uiMap.selectedObject.name, location: "[" + posX + "," + posY + "]"});
        console.log("Added waypoint");
    }
}

function canvasKeypressHandler(event) {
    //console.log(event)
    // if an space key and an object is selected, remove the last waypoint
    if (event.key == "Backspace" && uiMap.selectedObject != null) {
        uiMap.selectedObject.waypoints.pop();
        if (uiMap.stage != 2) {
            fetch("remove-waypoint?id=" + uiMap.selectedObject.index);
        }
        log({action: "remove-waypoint", target: uiMap.selectedObject.name});
        console.log("Removed waypoint");
    }

    // if a enter key and an object is selected, remove the last waypoint
    if (event.key == " " && uiMap.selectedObject != null) {
        log({action: "deselect-vehicle", target: uiMap.selectedObject.name});
        console.log("Deselected vehicle");
        uiMap.selectedObject = null;
    }
}