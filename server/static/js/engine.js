// functions including setIntervals and timeouts for the overall game engine

function initEngine() {
    // have the map update camera images almost constantly
    image_counter = 0;
    cam1 = document.getElementById("cam1_image");
    cam2 = document.getElementById("cam2_image");
    cam3 = document.getElementById("cam3_image");
    cam4 = document.getElementById("cam4_image");

    window.setInterval(function() {
        // update the cameras
        fetch("cams").then(data => data.text()).then(data => {
            data = JSON.parse(data);

            cam1.src = "data:image/png;base64, " + data[0];
            cam2.src = "data:image/png;base64, " + data[1];
            cam3.src = "data:image/png;base64, " + data[2];
            cam4.src = "data:image/png;base64, " + data[3];
        });
    }, 5000);

    window.setInterval(function() {
        // update the robot positions
        fetch("positions").then(data => data.text()).then(data => {
            data = JSON.parse(data);

            uiMap.uiObjects[0].x = data[0][0] * uiMap.mapCanvas.width;
            uiMap.uiObjects[0].y = data[0][1] * uiMap.mapCanvas.height;
            uiMap.uiObjects[1].x = data[1][0] * uiMap.mapCanvas.width;
            uiMap.uiObjects[1].y = data[1][1] * uiMap.mapCanvas.height;
            uiMap.uiObjects[2].x = data[2][0] * uiMap.mapCanvas.width;
            uiMap.uiObjects[2].y = data[2][1] * uiMap.mapCanvas.height;
            uiMap.uiObjects[3].x = data[3][0] * uiMap.mapCanvas.width;
            uiMap.uiObjects[3].y = data[3][1] * uiMap.mapCanvas.height;
        });

        fetch("get-waypoints").then(data => data.text()).then(data => {
            data = JSON.parse(data);

            uiMap.uiObjects[0].waypoints = data[0];
            uiMap.uiObjects[1].waypoints = data[1];
            uiMap.uiObjects[2].waypoints = data[2];
            uiMap.uiObjects[3].waypoints = data[3];
        });
    }, 5000);

    // add a mount event handler for the canvas
    window.addEventListener("mousedown", canvasMouseHandler);

    // add a keypress listener
    window.addEventListener("keydown", canvasKeypressHandler);
}