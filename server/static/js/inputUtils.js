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
            break;
        }
    }

    // if no vehicles were clicked, add a waypoint if applicable
    if (!clickedVehicle && uiMap.selectedObject != null && event.button == 0) {
        uiMap.selectedObject.waypoints.push([canvasX / uiMap.mapCanvas.width, canvasY / uiMap.mapCanvas.height]);
        console.log(" added waypoint, ", uiMap.selectedObject.waypoints);
    }
}

function canvasKeypressHandler(event) {
    console.log(event)
    // if an space key and an object is selected, remove the last waypoint
    if (event.key == "Delete" && uiMap.selectedObject != null) {
        console.log("Removed waypoint");
        uiMap.selectedObject.waypoints.pop();
    }

    // if a enter key and an object is selected, remove the last waypoint
    if (event.key == " " && uiMap.selectedObject != null) {
        console.log("Deselected vehicle");
        uiMap.selectedObject = null;
    }
}