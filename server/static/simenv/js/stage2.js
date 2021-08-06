// Create the game objects for Stage 1
function initStage2() { 
    // log that the page has opened
    log({"stage": "2", "action": "opened-page"});

    // no cameras in this stage, so hide the camera feeds
    // document.getElementById("left-panel").style.display = "none";
    document.getElementById("cam2").style.display = "none";
    document.getElementById("cam4").style.display = "none";
    document.getElementById("left-panel").innerHTML = "";

    // set the uiMap to not use networks
    uiMap.networked = false;
    uiMap.stage = 2;
    uiMap.checkButtons = () => {};

    // initialize two UAVs and two UGVs
    uav1 = new Vehicle("uav");
    uav1.index = 1;
    uav1.name = "UAV1";
    uav1.color = "red";

    uav2 = new Vehicle("uav");
    uav2.index = 2;
    uav2.name = "UAV2";
    uav2.color = "red";

    uav3 = new Vehicle("uav");
    uav3.index = 3;
    uav3.name = "UAV3";
    uav3.color = "red";

    uav4 = new Vehicle("uav");
    uav4.index = 4;
    uav4.name = "UAV4";
    uav4.color = "red";


    ugv1 = new Vehicle("ugv");
    ugv1.index = 5;
    ugv1.name = "UGV1";
    ugv1.color = "blue";

    ugv2 = new Vehicle("ugv");
    ugv2.index = 6;
    ugv2.name = "UGV2";
    ugv2.color = "blue";

    ugv3 = new Vehicle("ugv");
    ugv3.index = 7;
    ugv3.name = "UGV3";
    ugv3.color = "blue";

    ugv4 = new Vehicle("ugv");
    ugv4.index = 8;
    ugv4.name = "UGV4";
    ugv4.color = "blue";


    // initialize five caches
    cache1 = new Cache();
    cache1.name = "Cache 1";
    cache1.x = .200 * uiMap.mapCanvas.width;
    cache1.y = .298 * uiMap.mapCanvas.height;    

    cache2 = new Cache();
    cache2.name = "Cache 2";
    cache2.x = .564 * uiMap.mapCanvas.width;
    cache2.y = .327 * uiMap.mapCanvas.height;

    cache3 = new Cache();
    cache3.name = "Cache 3";
    cache3.x = .900 * uiMap.mapCanvas.width;
    cache3.y = .374 * uiMap.mapCanvas.height;

    cache4 = new Cache();
    cache4.name = "Cache 4";
    cache4.x = .895 * uiMap.mapCanvas.width;
    cache4.y = .877 * uiMap.mapCanvas.height;

    cache5 = new Cache();
    cache5.name = "Cache 5";
    cache5.x = .078 * uiMap.mapCanvas.width;
    cache5.y = .758 * uiMap.mapCanvas.height;


    // initialize the base
    base1 = new Base();
    base1.x = .580 * uiMap.mapCanvas.width;
    base1.y = .987 * uiMap.mapCanvas.height;

    // add them to the UI Map
    uiMap.uiObjects.push(base1);  // PUSH THE BASE FIRST!! So the linking algorithm starts at the base

    uiMap.uiObjects.push(uav1);
    uiMap.uiObjects.push(uav2);
    uiMap.uiObjects.push(uav3);
    uiMap.uiObjects.push(uav4);

    uiMap.uiObjects.push(ugv1);
    uiMap.uiObjects.push(ugv2);
    uiMap.uiObjects.push(ugv3);
    uiMap.uiObjects.push(ugv4);

    uiMap.uiObjects.push(cache1);
    uiMap.uiObjects.push(cache2);
    uiMap.uiObjects.push(cache3);
    uiMap.uiObjects.push(cache4);
    uiMap.uiObjects.push(cache5);

    // sets the robot positions
    setInitialPositions();

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
    

    // initialize the end conditions
    uiMap.endCheck = stage2EndCheck;

    // initialize the simulated motion
    simMotion();

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment";

    var instructionsTop = document.getElementById("instructions-top");
    instructionsTop.innerHTML = "Extend the communication network from the \"Base\" to all five caches.";

    // set up the instructions
    //var instructionsLeft = document.getElementById("left-panel");

    //instructionsLeft.innerHTML = "Your task is to create a communications network from \"Base\" to all five caches. Each robot has a signal range, shown by a dashed circle around it. When signals overlap the robots are connected. Many robots can be connected in a chain to form a widespread network, but a robot can only move if it is part of a network that includes \"Base\". When all caches are connected to \"Base\" this stage will be complete.";
 
}


function stage2EndCheck() {
    // if all caches are connected, end
    if (uiMap.stage == 2 && !uiMap.cacheDisconnected) {
        return true;
    }
    return false;
}