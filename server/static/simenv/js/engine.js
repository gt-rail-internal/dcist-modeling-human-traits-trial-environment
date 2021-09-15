// functions including setIntervals and timeouts for the overall game engine

function initEngine() {
    // if the stage is 2, set the distance traveled to 0
    uiMap.distanceTraveled = [0, 0, 0, 0, 0, 0, 0, 0];    

    if (uiMap.networked) {
        // get positions the first time, replace with "init stage X"
        getPositions();

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
            uiMap.checkButtons();
        }, 200);

    }

    // log the distance traveled every 5 seconds
    window.setInterval(function() {
        log({stage: uiMap.stage, action: "distance-checkup", object: "distance-traveled:" + uiMap.distanceTraveled, locations: "locations:" + uiMap.robotLocations, cacheStates: "cacheStates:" + uiMap.cacheStates });
    }, 5000);

    // set up the adhoc network check, if on Stage 3
    if (uiMap.stage == 3) {
        window.setInterval(function() {
            manageAdHocRobots();
        }, 200);
    }

    // set up the instructions bar to allow resetting the map
    document.getElementById("instructions-top").onclick = () => {
        log({"stage": uiMap.stage, "action": "reset map"});
        window.setTimeout(() => {
            if (uiMap.stage == 0) {
                location.reload();
            }
        }, 500);
    }

    // set up the completion check
    checkConditions();

    // add a mount event handler for the canvas
    window.addEventListener("mousedown", canvasMouseHandler);

    // add a keypress listener
    window.addEventListener("keydown", canvasKeypressHandler);
}

function startReplay() {
    window.setInterval(() => {
        replayCycle();
    }, 100);
    uiMap.replay = true;
}

function replayCycle() {
    
}

// uses the server parameter robot_positions to set the initial robot positions
function setInitialPositions() {
    console.log("AAA", robot_positions, uiMap.uiObjects)
    for (let obj in uiMap.uiObjects) {
        console.log(obj, uiMap.uiObjects[obj].name)
        if (robot_positions.hasOwnProperty(uiMap.uiObjects[obj].name)) {
            uiMap.uiObjects[obj].x = robot_positions[uiMap.uiObjects[obj].name][0] * uiMap.mapCanvas.width;
            uiMap.uiObjects[obj].y = robot_positions[uiMap.uiObjects[obj].name][1] * uiMap.mapCanvas.height;

            idx = -1
            if (uiMap.uiObjects[obj].name == "UGV1") {
                idx = 0;
            } 
            if (uiMap.uiObjects[obj].name == "UGV2") {
                idx = 1;
            } 
            if (uiMap.uiObjects[obj].name == "UGV3") {
                idx = 2;
            } 
            if (uiMap.uiObjects[obj].name == "UGV4") {
                idx = 3;
            } 
            if (uiMap.uiObjects[obj].name == "UAV1") {
                idx = 4;
            }
            if (uiMap.uiObjects[obj].name == "UAV2") {
                idx = 5;
            }
            if (uiMap.uiObjects[obj].name == "UAV3") {
                idx = 6;
            }
            if (uiMap.uiObjects[obj].name == "UAV4") {
                idx = 7;
            }

            if (idx != -1) {
                uiMap.robotLocations[idx][0] = robot_positions[uiMap.uiObjects[obj].name][0] * uiMap.mapCanvas.width;
                uiMap.robotLocations[idx][1] = robot_positions[uiMap.uiObjects[obj].name][1] * uiMap.mapCanvas.width;
                uiMap.robotLocations[idx][2] = Math.PI/2;    
            }
            
            console.log("set", uiMap.uiObjects[obj].name)
        }
    }
}

function getCams() {
    // update the cameras
    fetch("cams").then(data => data.text()).then(data => {
        data = JSON.parse(data);

        //console.log("camdata", data)

        cam1.src = "data:image/jpg;base64, " + data[uiMap.camNames[0]];
        cam2.src = "data:image/jpg;base64, " + data[uiMap.camNames[1]];
        cam3.src = "data:image/jpg;base64, " + data[uiMap.camNames[2]];
        cam4.src = "data:image/jpg;base64, " + data[uiMap.camNames[3]];

        //console.log("got cams")
    });
}

function getPositions() {
    // update the robot positions
    fetch("positions").then(data => data.text()).then(data => {
        data = JSON.parse(data);

        //console.log("got positions", data)

        // update the ui objects
        for (let i in uiMap.uiObjects) {
            // if the position was given for an object
            if (data.hasOwnProperty(uiMap.uiObjects[i].name)) {
                // set it
                uiMap.uiObjects[i].oldX = data[uiMap.uiObjects[i].name][0] * uiMap.mapCanvas.width == uiMap.uiObjects[i].x ? uiMap.uiObjects[i].oldX : uiMap.uiObjects[i].x;
                uiMap.uiObjects[i].oldY = data[uiMap.uiObjects[i].name][1] * uiMap.mapCanvas.height == uiMap.uiObjects[i].y ? uiMap.uiObjects[i].oldY : uiMap.uiObjects[i].y;
                
                uiMap.uiObjects[i].x = data[uiMap.uiObjects[i].name][0] * uiMap.mapCanvas.width;
                uiMap.uiObjects[i].y = data[uiMap.uiObjects[i].name][1] * uiMap.mapCanvas.height;

                // get the distance this robot has traveled and add it to the list
                dist = distance([uiMap.uiObjects[i].oldX, uiMap.uiObjects[i].oldY], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);

                // if the distance is greater than 5, ignore it (impossible, indicates robot hasn't moved yet)
                if (dist > 5.0) {
                    continue;
                }

                let idx = -1

                // if not, add to the vehicle's distance traveled
                if (uiMap.uiObjects[i].name == "UGV1") {
                    idx = 0;
                } 
                if (uiMap.uiObjects[i].name == "UGV2") {
                    idx = 1;
                } 
                if (uiMap.uiObjects[i].name == "UGV3") {
                    idx = 2;
                } 
                if (uiMap.uiObjects[i].name == "UGV4") {
                    idx = 3;
                } 
                if (uiMap.uiObjects[i].name == "UAV1") {
                    idx = 4;
                }
                if (uiMap.uiObjects[i].name == "UAV2") {
                    idx = 5;
                }
                if (uiMap.uiObjects[i].name == "UAV3") {
                    idx = 6;
                }
                if (uiMap.uiObjects[i].name == "UAV4") {
                    idx = 7;
                }
                
                if (idx != -1) {
                    uiMap.distanceTraveled[idx] = uiMap.distanceTraveled[idx] + dist;
                    uiMap.robotLocations[idx][0] = uiMap.uiObjects[i].x;
                    uiMap.robotLocations[idx][1] = uiMap.uiObjects[i].y;
                    uiMap.robotLocations[idx][2] = -Math.atan2(uiMap.uiObjects[i].y - uiMap.uiObjects[i].oldY, uiMap.uiObjects[i].oldX - uiMap.uiObjects[i].x) + Math.PI/2;
                }
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
            if (data.hasOwnProperty(uiMap.uiObjects[i].name)) {
                let old_waypoints = uiMap.uiObjects[i].waypoints;
                // if the robot is now stopped, log that it is idle
                if (data[uiMap.uiObjects[i].name].length == 0 && old_waypoints.length > 0) {
                    log({stage: uiMap.stage, action: "robot " + uiMap.uiObjects[i].name + " is idle"});
                }
                // if the robot is no longer stopped, log that it is not idle
                if (data[uiMap.uiObjects[i].name].length > 0 && old_waypoints.length == 0) {
                    log({stage: uiMap.stage, action: "robot " + uiMap.uiObjects[i].name + " is no longer idle"});
                }
                uiMap.uiObjects[i].waypoints = data[uiMap.uiObjects[i].name];

                //console.log("updated", uiMap.uiObjects[i].name,  uiMap.uiObjects[i].waypoints);
            }
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
        //console.log("TIMEOUT CHECK", checkTimeout());

        // if the stage is complete, return
        if (uiMap.stageComplete) {
            return;
        }

        // if the time limit has completed, time out
        else if (checkTimeout()) {
            log({stage: uiMap.stage, action: "stage-complete", object: "distance-traveled:" + uiMap.distanceTraveled, locations: "locations:" + uiMap.robotLocations})
            console.log("Timeout!");
            stageComplete();
            return;
        }

        // if the end condition is met, victory
        else if (uiMap.endCheck()) {
            log({stage: uiMap.stage, action: "stage-complete", object: "distance-traveled:" + uiMap.distanceTraveled, locations: "locations:" + uiMap.robotLocations})
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

    let oldNumConnected = 0;

    let lastUpdate = date.getTime();  // record the last update time 
    window.setInterval(() => {
        // if the stage is complete, don't move
        if (uiMap.stageComplete) {
            return;
        }

        let now = new Date().getTime();

        uiMap.cacheDisconnected = false;

        let numConnected = 0;

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

                let step_dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))

                // record the distance traveled
                let robot_name = uiMap.uiObjects[i].name;

                let robot_id = robot_name == "UGV1" ? 0 : robot_name == "UGV2" ? 1 : robot_name == "UGV3" ? 2 : robot_name == "UGV4" ? 3 : robot_name == "UAV1" ? 4 : robot_name == "UAV2" ? 5 : robot_name == "UAV3" ? 6 : robot_name == "UAV4" ? 7 : -1;
                
                // move the object
                uiMap.uiObjects[i].oldX = uiMap.uiObjects[i].x;
                uiMap.uiObjects[i].oldY = uiMap.uiObjects[i].y;

                uiMap.uiObjects[i].x = uiMap.uiObjects[i].x + dx;
                uiMap.uiObjects[i].y = uiMap.uiObjects[i].y + dy;

                // record the robot locations
                if (robot_id != -1) {  // only record valid robots
                    uiMap.robotLocations[robot_id][2] = -Math.atan2(uiMap.uiObjects[i].y - uiMap.robotLocations[robot_id][1], uiMap.uiObjects[i].x - uiMap.robotLocations[robot_id][0]) + Math.PI/2;
                    uiMap.robotLocations[robot_id][0] = uiMap.uiObjects[i].x;
                    uiMap.robotLocations[robot_id][1] = uiMap.uiObjects[i].y;
                    uiMap.distanceTraveled[robot_id] += step_dist;
                }

                // if close to the waypoint, remove the waypoint and log if the robot is at its end
                if (distance([uiMap.uiObjects[i].x, uiMap.uiObjects[i].y], [uiMap.uiObjects[i].waypoints[0][0] * uiMap.mapCanvas.width, uiMap.uiObjects[i].waypoints[0][1] * uiMap.mapCanvas.height]) < 5) {
                    uiMap.uiObjects[i].waypoints.shift();
                    if (uiMap.uiObjects[i].waypoints.length == 0) {
                        log({stage: uiMap.stage, action: "robot " + uiMap.uiObjects[i].name + " idle"});
                    }
                }
            }

            // check cache connections
            if (uiMap.uiObjects[i].constructor.name == "Cache") {
                // check if cache is connected to base
                if (uiMap.adHocLock) {
                    // get the ID of the cache
                    let cacheId = uiMap.uiObjects[i].name == "Cache 1" ? 0 : uiMap.uiObjects[i].name == "Cache 2" ? 1 : uiMap.uiObjects[i].name == "Cache 3" ? 2 : uiMap.uiObjects[i].name == "Cache 4" ? 3 : uiMap.uiObjects[i].name == "Cache 5" ? 4 : -1;
                    
                    // if connected, great, set a flag
                    if (robotConnectedToBase(uiMap.uiObjects[i].name)) {
                        uiMap.uiObjects[i].connected = true;
                        numConnected += 1;
                        
                        // mark the cache as connected
                        uiMap.cacheStates[cacheId] = 1;

                        // update the training counter
                        if (uiMap.stage == 0) {
                            trainingReachCache += 1;
                            checkTraining();
                        }
                    }
                    else {
                        // mark the cache as disconnected
                        uiMap.cacheStates[cacheId] = 0;
                        
                        uiMap.uiObjects[i].connected = false;
                        uiMap.cacheDisconnected = true;
                    }
                }
            }
        }

        // if the number connected is different than the running number, send a log
        if (numConnected != oldNumConnected) {
            log({stage: uiMap.stage, action: "cache connected count", object: numConnected})
        }
        oldNumConnected = numConnected;

        lastUpdate = now;

    }, 100);
}

// checks whether robots are connected to the base, if not, flags them as disconnected
function manageAdHocRobots() {
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
                    if (!uiMap.uiObjects[i].nameAttention && uiMap.stage != 2) {
                        fetch("/remove-all-waypoints?id=" + uiMap.uiObjects[i].name)
                    }

                    // flag as needing attention
                    uiMap.uiObjects[i].nameAttention = true;
                    continue;
                }
            }
        }
    }
}


// helper function for getting the distance between two [x,y] arrays
function distance(a, b) {
    return Math.sqrt(Math.pow((a[0] - b[0]), 2) + Math.pow((a[1] - b[1]), 2));
}


// determines whether a robot is ad hoc connected to the base (true) or not (false)
function robotConnectedToBase(robot) {

    // if Stage 1, don't worry about it
    
    // if Test Stage or Stage 2, do BFS to check if connects to a base
    if (uiMap.stage == 0 || uiMap.stage == 2 || uiMap.stage == 3) {
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
            if (uiMap.uiObjects[i].constructor.name != "Vehicle" && uiMap.uiObjects[i].constructor.name != "Base" && uiMap.uiObjects[i].constructor.name != "Cache" && uiMap.uiObjects[i].constructor.name != "Router") {
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
    log({stage: uiMap.stage, action: "stage-complete", object: "distance-traveled:" + uiMap.distanceTraveled})

    document.getElementById("instructions-top").innerHTML = "This stage has now ended! Please notify the researcher.";
    document.getElementById("instructions-top").style.backgroundColor = "lightgreen";

    // after half a second (for the log message to go through), redirect
    window.setTimeout(() => {
        // set the ui map to complete
        uiMap.stageComplete = true;

        // set all vehicle names to "Complete"
        for (i in uiMap.uiObjects) {
            uiMap.uiObjects[i].name = "Complete";
        }

        document.getElementById("instructions-top").innerHTML = "This stage has now ended! Please notify the researcher.";
        document.getElementById("instructions-top").style.backgroundColor = "lightgreen";

        // if on the training stage, move to the next stage (COMMENTED OUT BECAUSE WE ARE LINKING PLAYERS TO STAGES DIRECTLY)
        if (false && uiMap.stage == 0) {
        //    window.location.href = "/stage?workerId=" + uiMap.workerID + "&stage=" + mission + "&mission=" + mission;
        }
        else {
            log({stage: uiMap.stage, action: "stage-complete", object: "distance-traveled:" + uiMap.distanceTraveled});
            alert("Please notify that researcher that you have completed this stage");
            //window.location.href = "/portal?workerId=" + uiMap.workerID + "&pageFrom=" + 3 + "&success=1" + "&mission=" + mission;
        }

    }, 1000);
}

function log(data) {
    data["worker-id"] = uiMap.workerID;  // include the worker ID
    data["stage"] = uiMap.stage;
    console.log("log", data, uiMap.workerID);
    return fetch("/logging", {method: "POST", body: JSON.stringify(data)});
}

function startStage() {
    stageStarted = true;
    startTime = new Date().getTime() / 1000;
}

function checkTimeout() {
    // if the stage has not started, do not complete yet
    if (!stageStarted) {
        return false;
    }

    let now = new Date().getTime() / 1000;
    let timeout = mission != 1 || uiMap.stage != 0 ? 60 * 10 : 60 * 5;

    if (now - startTime > timeout) {
        return true;
    }

    return false;
}