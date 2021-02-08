// handles the canvas mouse events
function canvasMouseHandler(event) {
    // if the mouse click is outside the range of the canvas, ignore it
    if (event.originalTarget != document.getElementById("uimap-canvas") && event.target != document.getElementById("uimap-canvas")) {
        return;
    }

    var canvasX = event.clientX - (event.originalTarget ? event.originalTarget.offsetLeft : event.target.offsetLeft);
    var canvasY = event.clientY - (event.originalTarget ? event.originalTarget.offsetTop : event.target.offsetTop);
    var canvasClick = [canvasX, canvasY];

    clickedVehicle = false;

    // check if any vehicles were clicked
    for (i in uiMap.uiObjects) {
        if (uiMap.distance(canvasClick, [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < uiMap.uiObjects[i].scale * .86) {
            console.log("Selected vehicle");
            uiMap.selectedObject = uiMap.uiObjects[i];
            clickedVehicle = true;
            clickedVehicleIndex = i;
            break;
        }
    }

    // if no vehicles were clicked, add a waypoint if applicable
    if (!clickedVehicle && uiMap.selectedObject != null && event.button == 0) {
        posX = canvasX / uiMap.mapCanvas.width;
        posY = canvasY / uiMap.mapCanvas.height;
        uiMap.selectedObject.waypoints.push([posX, posY]);
        fetch("add-waypoint?id=" + uiMap.selectedObject.index + "&x=" + posX + "&y=" + posY)
        console.log(" added waypoint, ", uiMap.selectedObject.waypoints);
    }
}

function canvasKeypressHandler(event) {
    console.log(event)
    // if an space key and an object is selected, remove the last waypoint
    if (event.key == "Backspace" && uiMap.selectedObject != null) {
        console.log("Removed waypoint");
        uiMap.selectedObject.waypoints.pop();
        fetch("remove-waypoint?id=" + uiMap.selectedObject.index);
    }

    // if a enter key and an object is selected, remove the last waypoint
    if (event.key == " " && uiMap.selectedObject != null) {
        console.log("Deselected vehicle");
        uiMap.selectedObject = null;
    }
}