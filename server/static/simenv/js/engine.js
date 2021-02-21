// functions including setIntervals and timeouts for the overall game engine

function initEngine() {
    // get positions the first time, replace with "init stage X"
    getPositions();

    if (uiMap.networked) {
        // have the map update camera images almost constantly
        image_counter = 0;
        cam1 = document.getElementById("cam1_image");
        cam2 = document.getElementById("cam2_image");
        cam3 = document.getElementById("cam3_image");
        cam4 = document.getElementById("cam4_image");
        window.setInterval(function() {
            getCams();
            getPositions();
            getWaypoints();
        }, 5000);
    }

    // set up the completion check
    checkConditions();

    // add a mount event handler for the canvas
    window.addEventListener("mousedown", canvasMouseHandler);

    // add a keypress listener
    window.addEventListener("keydown", canvasKeypressHandler);
}

function getCams() {
    // update the cameras
    fetch("cams").then(data => data.text()).then(data => {
        data = JSON.parse(data);

        cam1.src = "data:image/png;base64, " + data[0];
        cam2.src = "data:image/png;base64, " + data[1];
        cam3.src = "data:image/png;base64, " + data[2];
        cam4.src = "data:image/png;base64, " + data[3];
    });
}

function getPositions() {
    // update the robot positions
    fetch("positions").then(data => data.text()).then(data => {
        data = JSON.parse(data);

        // update the ui objects
        for (let i in uiMap.uiObjects) {
            //console.log(uiMap.uiObjects[i].name, data)
            // if the position was given for an object
            if (data.hasOwnProperty(uiMap.uiObjects[i].name)) {
                // set it
                uiMap.uiObjects[i].x = data[uiMap.uiObjects[i].name][0] * uiMap.mapCanvas.width;
                uiMap.uiObjects[i].y = data[uiMap.uiObjects[i].name][1] * uiMap.mapCanvas.height;
            }
        }
    });
}

// retrieves waypoints from the server
function getWaypoints() {
    // update the robot waypoints
    fetch("get-waypoints").then(data => data.text()).then(data => {
        data = JSON.parse(data);

        for (var i in uiMap.uiObjects) {
            console.log(">>>", uiMap.uiObjects[i].name)
            uiMap.uiObjects[i].waypoints = data[uiMap.uiObjects[i].name];
        }

    });
}


// checks if a waypoint is valid
function checkValidWaypoint(prior_waypoint, waypoint) {
    // check whether the waypoint intersects any shapes

    for (i in uiMap.uiObstacles) {
        if (uiMap.uiObstacles[i].checkWaypointIntersects([prior_waypoint, waypoint])) {
            return false;
        }
    }

    return true;
}

function checkConditions() {
    window.setInterval(() => {
        // if the stage is complete, return
        if (uiMap.stageComplete) {
            return;
        }

        // if the time limit has completed, time out
        if (checkTimeout()) {
            console.log("Timeout!");
            stageComplete();
            return;
        }

        // if the end condition is met, victory
        if (uiMap.endCheck()) {
            console.log("VICTORY!!");
            uiMap.stageVictory = true;
            stageComplete();
            return;
        }
    }, 250);
}

// runs vehicle motion
function simMotion() {
    let date = new Date();

    let lastUpdate = date.getTime();  // record the last update time 
    window.setInterval(() => {
        // if the stage is complete, don't move
        if (uiMap.stageComplete) {
            return;
        }

        let now = new Date().getTime();

        uiMap.cacheDisconnected = false;

        // move vehicles closer to their goals
        for (let i in uiMap.uiObjects) {
            if (uiMap.uiObjects[i].constructor.name == "Vehicle") {
                // check if robot is connected via adhoc, if applicable
                if (uiMap.adHocLock) {
                    // if connected, great, the robot is not locked
                    if (robotConnectedToBase(uiMap.uiObjects[i].name)) {
                        uiMap.uiObjects[i].nameAttention = false;
                    }
                    // if not connected
                    else{
                        // remove waypoints
                        uiMap.uiObjects[i].waypoints = [];

                        // if the object was not disconnected before, send a waypoint clearing message
                        if (!uiMap.uiObjects[i].nameAttention) {
                            fetch("/remove-all-waypoints?id=" + uiMap.uiObjects[i].name)
                        }

                        // flag as needing attention
                        uiMap.uiObjects[i].nameAttention = true;

                        // update the training counter
                        if (uiMap.stage == 0) {
                            trainingDisconnectRobot += 1;
                            console.log("disconnect")
                            checkTraining();
                        }

                        continue;
                    }
                }

                // if not moving the robot, continue to the next robot
                if (uiMap.uiObjects[i].waypoints.length == 0) {
                    continue;
                }

                // calculate how to move the object in x/y
                let dx = uiMap.uiObjects[i].waypoints[0][0] * uiMap.mapCanvas.width - uiMap.uiObjects[i].x;
                let dy = uiMap.uiObjects[i].waypoints[0][1] * uiMap.mapCanvas.height - uiMap.uiObjects[i].y;
                let normalization = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy,2));

                dx = (now - lastUpdate) * uiMap.uiObjects[i].speed * dx / normalization / 1000;  // normalize and account for speed
                dy = (now - lastUpdate) * uiMap.uiObjects[i].speed * dy / normalization / 1000;  // normalize and account for speed

                // move the object
                uiMap.uiObjects[i].x = uiMap.uiObjects[i].x + dx;
                uiMap.uiObjects[i].y = uiMap.uiObjects[i].y + dy;

                // if close to the waypoint, remove the waypoint
                if (distance([uiMap.uiObjects[i].x, uiMap.uiObjects[i].y], [uiMap.uiObjects[i].waypoints[0][0] * uiMap.mapCanvas.width, uiMap.uiObjects[i].waypoints[0][1] * uiMap.mapCanvas.height]) < 5) {
                    uiMap.uiObjects[i].waypoints.shift();
                }
            }

            // check cache connections
            if (uiMap.uiObjects[i].constructor.name == "Cache") {
                // check if cache is connected to base
                if (uiMap.adHocLock) {
                    // if connected, great, set a flag
                    if (robotConnectedToBase(uiMap.uiObjects[i].name)) {
                        uiMap.uiObjects[i].connected = true;

                        // update the training counter
                        if (uiMap.stage == 0) {
                            trainingReachCache += 1;
                            checkTraining();
                        }
                    }
                    else {
                        uiMap.uiObjects[i].connected = false;
                        uiMap.cacheDisconnected = true;
                    }
                }
            }
        }
        lastUpdate = now;

    }, 100);
}


// helper function for getting the distance between two [x,y] arrays
function distance(a, b) {
    return Math.sqrt(Math.pow((a[0] - b[0]), 2) + Math.pow((a[1] - b[1]), 2));
}


// determines whether a robot is ad hoc connected to the base (true) or not (false)
function robotConnectedToBase(robot) {

    // if Stage 1, don't worry about it
    
    // if Test Stage or Stage 2, do BFS to check if connects to a base
    if (uiMap.stage == 0 || uiMap.stage == 2) {
        result = BFS("Base", robot);
        //console.log(">>>", generateBFSTree(), result)
        return result.length > 0 ? true : false;
    }

    return true;  // return true if all else fails
}


// Breadth First Search implementation
// adapted from https://levelup.gitconnected.com/67ae4653dbec
function BFS(rootNode, searchValue) {
	// make a queue array
	let queue = [];
    let seen = [];
  	let path = [];

	// populate it with the node that will be the root of your search
    // get the index of the root node
    for (i in uiMap.uiObjects) {
        if (uiMap.uiObjects[i].name == rootNode) {
            queue.push(i);
            break;
        }
    }

	// search the queue until it is empty
	while (queue.length > 0) {
		// assign the top of the queue to variable currentNode
		let currentNode = queue[0];
		path.push(currentNode);

		// if currentNode is the node we're searching for, break & alert
		if (uiMap.uiObjects[currentNode].name === searchValue) {
			return path;
		}

		// add each child to the queue
        for (i in uiMap.uiObjects) {
            // ignore objects that are not vehicles or bases
            if (uiMap.uiObjects[i].constructor.name != "Vehicle" && uiMap.uiObjects[i].constructor.name != "Base" && uiMap.uiObjects[i].constructor.name != "Cache") {
                continue;
            }
        
            let dist = distance(uiMap.uiObjects[currentNode].getLoc(), uiMap.uiObjects[i].getLoc());
            if (!seen.includes(i) && dist < uiMap.uiObjects[currentNode].adHocRadius + uiMap.uiObjects[i].adHocRadius) {
                queue.push(i);
            }
        }

        // add the current node to the explored nodes
        seen.push(currentNode);

        // remove the currentNode from the queue.
		queue.shift();
	}

    return [];
};


function stageComplete() {
    // log it
    log({stage: uiMap.stage, action: "stage-complete"})

    // set the instructions
    topBar = document.getElementById("instructions-top");
    topBar.classList = "instructions green-success"
    topBar.innerHTML = uiMap.stageVictory ? "🎉 Congratulations! You have completed this stage. <b>Click here to continue</b>." : "This stage has ended. <b>Click here to continue</b>.";
    topBar.onclick = () => {
        topBar.innerHTML = "Redirecting you to the experiment portal"
        // if on the training stage, move to Stage 2
        if (uiMap.stage == 0) {
            window.location.href = "/stage?workerId=" + uiMap.workerID + "&stage=2";
        }
        else {
            window.location.href = "/portal?workerId=" + uiMap.workerID + "&pageFrom=" + 3 + "&success=1";
        }
    }

    // set the ui map to complete
    uiMap.stageComplete = true;

    // set all vehicle names to "Complete"
    for (i in uiMap.uiObjects) {
        uiMap.uiObjects[i].name = "Complete";
    }

}


function log(data) {
    data["worker-id"] = uiMap.workerID;  // include the worker ID
    data["stage"] = uiMap.stage;
    //console.log("log", data, uiMap.workerID);
    return fetch("/logging", {method: "POST", body: JSON.stringify(data)});
}

function checkTimeout() {
    now = new Date().getTime() / 1000;
    timeout = 60 * 10;

    if (now - startTime > timeout) {
        return true;
    }

    return false;
}