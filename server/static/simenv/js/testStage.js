var trainingSelectRobot = 0;
var trainingAddWaypoints = 0;
var trainingRemoveWaypoints = 0;
var trainingDeselectRobot = 0;
var trainingStopRobot = 0;
var trainingDisconnectRobot = 0;
var trainingReachCache = 0;

// Create the game objects for the Test Stage
function initTestStage() { 
    // log that the page has opened
    log({"stage": "0", "action": "opened-page"});

    // no cameras in this stage, so hide the camera feeds
    // document.getElementById("left-panel").style.display = "none";
    document.getElementById("cam2").style.display = "none";
    document.getElementById("cam4").style.display = "none";
    document.getElementById("left-panel").innerHTML = "";

    // set the uiMap to not use networks
    uiMap.networked = false;
    uiMap.stage = 0;
    uiMap.displayAdHocRanges = mission == 1 ? false : true;
    uiMap.adHocLock = mission == 1 ? false : true;

    // initialize one UAVs and one UGVs
    uav1 = new Vehicle("uav");
    uav1.index = 1;
    uav1.name = "UAV1";
    uav1.color = "red";
    uav1.x = .522 * uiMap.mapCanvas.width;
    uav1.y = .970 * uiMap.mapCanvas.height;

    ugv1 = new Vehicle(mission != "1" ? "ugv" : "uav");
    ugv1.index = 2;
    ugv1.name = mission != "1" ? "UGV1" : "UAV2";
    ugv1.color = "blue";
    ugv1.x = .641 * uiMap.mapCanvas.width;
    ugv1.y = .970 * uiMap.mapCanvas.height;

    // initialize one cache
    cache1 = new Cache();
    cache1.name = "Cache 1";
    cache1.x = .5 * uiMap.mapCanvas.width;
    cache1.y = .5 * uiMap.mapCanvas.height;    


    // initialize the base
    base1 = new Base();
    base1.x = .580 * uiMap.mapCanvas.width;
    base1.y = .987 * uiMap.mapCanvas.height;

    // add them to the UI Map
    uiMap.uiObjects.push(base1);  // PUSH THE BASE FIRST!! So the linking algorithm starts at the base
    uiMap.uiObjects.push(uav1);
    uiMap.uiObjects.push(ugv1);
    uiMap.uiObjects.push(cache1);

    // add the map collisions
    northWarehouse = new MapUIObstacle();
    northWarehouse.shape = [[.600, .297],
                            [.457, .383],
                            [.424, .333],
                            [.570, .245]];

    northResidential = new MapUIObstacle();
    northResidential.shape = [[.764, .041],
                              [.864, .150],
                              [.806, .199],
                              [.711, .094]];

    westWarehouse = new MapUIObstacle();
    westWarehouse.shape = [[.250, .608],
                           [.334, .740],
                           [.330, .749],
                           [.281, .780],
                           [.193, .664]];

    southResidential = new MapUIObstacle();
    southResidential.shape = [[.443, .834],
                              [.445, .850],
                              [.410, .981],
                              [.354, .970],
                              [.390, .823]];

    westHouse = new MapUIObstacle();
    westHouse.shape = [[.420, .568],
                       [.402, .582],
                       [.388, .570],
                       [.372, .584],
                       [.350, .554],
                       [.357, .552],
                       [.350, .542],
                       [.382, .515],
                       [.388, .524],
                       [.394, .518],
                       [.415, .548],
                       [.408, .555]];

    eastHouse = new MapUIObstacle();
    eastHouse.shape = [[.768, .550],
                       [.784, .535],
                       [.792, .547],
                       [.818, .525],
                       [.814, .521],
                       [.825, .510],
                       [.802, .484],
                       [.791, .481],
                       [.782, .484],
                       [.775, .481],
                       [.754, .500],
                       [.764, .514],
                       [.750, .527]];

    southeastHouse = new MapUIObstacle();
    southeastHouse.shape = [[.662, .721],
                           [.660, .734],
                           [.664, .738],
                           [.652, .765],
                           [.641, .762],
                           [.635, .774],
                           [.612, .767],
                           [.617, .757],
                           [.600, .750],
                           [.612, .711],
                           [.618, .711],
                           [.625, .702],
                           [.637, .707],
                           [.640, .701],
                           [.650, .704],
                           [.648, .710],
                           [.655, .712]];

    southwestHouse = new MapUIObstacle();
    southwestHouse.shape = [[.552, .738],
                            [.528, .735],
                            [.530, .725],
                            [.512, .720],
                            [.517, .682],
                            [.522, .682],
                            [.525, .671],
                            [.537, .671],
                            [.540, .665],
                            [.551, .668],
                            [.551, .672],
                            [.562, .674],
                            [.568, .684],
                            [.571, .694],
                            [.568, .725],
                            [.554, .725]];
                   

    uiMap.uiObstacles.push(northWarehouse);
    uiMap.uiObstacles.push(northResidential);
    uiMap.uiObstacles.push(westWarehouse);
    uiMap.uiObstacles.push(southResidential);
    uiMap.uiObstacles.push(westHouse);
    uiMap.uiObstacles.push(eastHouse);
    uiMap.uiObstacles.push(southwestHouse);
    uiMap.uiObstacles.push(southeastHouse);

    // initialize the simulated motion
    simMotion();

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment - Trial Stage";
    title.style.backgroundColor = "lightblue";

    var instructionsTop = document.getElementById("instructions-top");
    instructionsTop.innerHTML = "Familiarize yourself with controlling robots. Try to complete the goals on the left side. If you get stuck or want to reset the map, <b>click here</b>.";
    instructionsTop.style.width = uiMap.mapCanvas.width + "px";
    instructionsTop.onclick = () => {
        log({"stage": "0", "action": "reset map"});
        window.setTimeout(() => {
            location.reload();
        }, 500);
    }


    document.getElementById("instructions-giveup").innerHTML = mission == 1 ? "After 5 minutes you will move on to the full experiment. Resetting the map will reset this timeout." : document.getElementById("instructions-giveup").innerHTML + " Resetting the map will reset this timeout."

    // set up the instructions
    var instructionsLeft = document.getElementById("left-panel");
    instructionsLeft.style.justifyContent = "flex-start";

    let taskGoals = document.createElement("div");
    taskGoals.classList = "";
    taskGoals.innerHTML = "Goals:";
    taskGoals.style.marginTop = "30%";
    taskGoals.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskGoals);

    let taskSelectRobot = document.createElement("div");
    taskSelectRobot.classList = "grey-instructions";
    taskSelectRobot.innerHTML = "Select a robot";
    taskSelectRobot.id = "select-robot-div";
    taskSelectRobot.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskSelectRobot);

    let taskSetWaypoints = document.createElement("div");
    taskSetWaypoints.classList = "grey-instructions";
    taskSetWaypoints.innerHTML = "Set three waypoints";
    taskSetWaypoints.id = "set-waypoints-div";
    taskSetWaypoints.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskSetWaypoints);

    let taskRemoveWaypoints = document.createElement("div");
    taskRemoveWaypoints.classList = "grey-instructions";
    taskRemoveWaypoints.innerHTML = "Remove a waypoint";
    taskRemoveWaypoints.id = "remove-waypoints-div";
    taskRemoveWaypoints.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskRemoveWaypoints);

    let taskDeselectRobot = document.createElement("div");
    taskDeselectRobot.classList = "grey-instructions";
    taskDeselectRobot.innerHTML = "Deselect a robot";
    taskDeselectRobot.id = "deselect-robot-div";
    taskDeselectRobot.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskDeselectRobot);

    let taskStopRobot = document.createElement("div");
    taskStopRobot.classList = "grey-instructions";
    taskStopRobot.innerHTML = "Stop a robot (remove all waypoints while it is moving)";
    taskStopRobot.id = "stop-robot-div";
    taskStopRobot.style.marginBottom = "10%";
    instructionsLeft.appendChild(taskStopRobot);

    let taskDisconnect = document.createElement("div");
    taskDisconnect.classList = "grey-instructions";
    taskDisconnect.innerHTML = "Disconnect a robot (move it out of range)";
    taskDisconnect.id = "disconnect-robot-div";
    taskDisconnect.style.marginBottom = "10%";
    if (mission != "1") {  // if the mission is the first stage, don't care about network connectivity
        instructionsLeft.appendChild(taskDisconnect);
    }

    let taskCache = document.createElement("div");
    taskCache.classList = "grey-instructions";
    taskCache.innerHTML = "Extend the signal network to the Cache<img src='/static/simenv/img/extended.png' style='width: 100%'>";
    taskCache.id = "reach-cache-div";
    taskCache.style.marginBottom = "10%";
    if (mission != "1") {  // if the mission is the first stage, don't care about network connectivity
        instructionsLeft.appendChild(taskCache);
    }
    
    uiMap.training = true;
    uiMap.endCheck = testStageEndCheck;
}

// updates the training goals backgrounds
function checkTraining() {
    if (trainingSelectRobot > 0) {
        document.getElementById("select-robot-div").style.backgroundColor = "palegreen";
    }
    if (trainingAddWaypoints >= 3) {
        document.getElementById("set-waypoints-div").style.backgroundColor = "palegreen";
    }
    if (trainingRemoveWaypoints > 0) {
        document.getElementById("remove-waypoints-div").style.backgroundColor = "palegreen";
    }
    if (trainingDeselectRobot > 0) {
        document.getElementById("deselect-robot-div").style.backgroundColor = "palegreen";
    }
    if (trainingStopRobot > 0) {
        document.getElementById("stop-robot-div").style.backgroundColor = "palegreen";
    }
    if (trainingDisconnectRobot > 0 && uiMap.interacted && mission != 1) {
        document.getElementById("disconnect-robot-div").style.backgroundColor = "palegreen";
    }
    if (trainingReachCache > 0 && mission != 1) {
        document.getElementById("reach-cache-div").style.backgroundColor = "palegreen";
    }
}


// check whether the end conditions are met
function testStageEndCheck() {
    if (mission != 1 && trainingSelectRobot > 0 && trainingAddWaypoints >= 3 && trainingRemoveWaypoints > 0 && trainingDeselectRobot > 0 && trainingStopRobot > 0 && trainingDisconnectRobot > 0 && uiMap.interacted && trainingReachCache > 0) {
        return true;
    }

    return false;
}