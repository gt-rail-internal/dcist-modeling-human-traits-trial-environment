// Create the game objects for Stage 1
function initStage2() { 
    // no cameras in this stage, so hide the camera feeds
    // document.getElementById("left-panel").style.display = "none";
    document.getElementById("right-panel").innerHTML = "";

    // set the uiMap to not use networks
    uiMap.networked = false;
    uiMap.stage = 2;

    // initialize two UAVs and two UGVs
    uav1 = new Vehicle("uav");
    uav1.index = 1;
    uav1.name = "UAV 1";
    uav1.color = "red";

    uav2 = new Vehicle("uav");
    uav2.index = 2;
    uav2.name = "UAV 2";
    uav2.color = "purple";

    uav3 = new Vehicle("uav");
    uav3.index = 3;
    uav3.name = "UAV 3";
    uav3.color = "blue";

    uav4 = new Vehicle("uav");
    uav4.index = 4;
    uav4.name = "UAV 4";
    uav4.color = "goldenrod";


    ugv1 = new Vehicle("ugv");
    ugv1.index = 5;
    ugv1.name = "UGV 1";
    ugv1.color = "black";

    ugv2 = new Vehicle("ugv");
    ugv2.index = 6;
    ugv2.name = "UGV 2";
    ugv2.color = "grey";

    ugv3 = new Vehicle("ugv");
    ugv3.index = 7;
    ugv3.name = "UGV 3";
    ugv3.color = "brown";

    ugv4 = new Vehicle("ugv");
    ugv4.index = 8;
    ugv4.name = "UGV 4";
    ugv4.color = "darkgreen";


    // initialize five caches
    cache1 = new Cache();
    cache1.x = 200;
    cache1.y = 500;    

    // initialize the base
    base1 = new Base();
    base1.x = 400;
    base1.y = 60;

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

    // add the map collisions
    lake = new MapUIObstacle();
    lake.shape = [[.694, .206],
                  [.784, .205],
                  [.815, .240],
                  [.850, .344],
                  [.851, .395],
                  [.829, .450],
                  [.802, .474],
                  [.755, .497],
                  [.725, .547],
                  [.675, .590],
                  [.637, .591],
                  [.604, .570],
                  [.575, .528],
                  [.580, .480],
                  [.588, .460],
                  [.648, .410],
                  [.671, .383],
                  [.677, .352],
                  [.666, .319],
                  [.647, .284],
                  [.651, .261],
                  [.674, .221]];

    mountains = new MapUIObstacle();
    mountains.shape = [[.165, .364],
                       [.245, .357],
                       [.387, .330],
                       [.322, .237],
                       [.288, .291],
                       [.257, .230],
                       [.220, .285],
                       [.197, .225]];

    uiMap.uiObstacles.push(lake);
    uiMap.uiObstacles.push(mountains);

    // initialize the simulated motion
    simMotion();

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment - Stage 2";

    // set up the instructions
    var instructionsLeft = document.getElementById("left-panel");

    instructionsLeft.innerHTML = "Your task is to create a communications network from \"Base\" to all five caches. Each robot has a signal range, shown by a dashed circle around it. When signals overlap the robots are connected. Many robots can be connected in a chain to form a widespread network, but a robot can only move if it is part of a network that includes \"Base\". When all caches are connected to \"Base\" this stage will be complete.";
    
    
    
}
