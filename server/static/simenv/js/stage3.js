// Create the game objects for Stage 1
function initStage3() { 

    // initialize the uiMap
    uiMap.networked = true;
    uiMap.stage = 3;
    uiMap.displayAdHocRanges = true;
    uiMap.adHocLock = true;
    uiMap.camNames = ["UGV1", "UGV2", "UGV3", "UGV4"]
    uiMap.checkButtons = checkButtons;

    // initialize two UAVs and two UGVs
    ugv1 = new Vehicle("ugv");
    ugv1.index = 1;
    ugv1.name = "UGV1";
    ugv1.color = "red";
    ugv1.adHocRadius = 50;

    ugv2 = new Vehicle("ugv");
    ugv2.index = 2;
    ugv2.name = "UGV2";
    ugv2.color = "purple";
    ugv2.adHocRadius = 50;
    

    ugv3 = new Vehicle("ugv");
    ugv3.index = 3;
    ugv3.name = "UGV3";
    ugv3.color = "blue";
    ugv3.adHocRadius = 50;

    ugv4 = new Vehicle("ugv");
    ugv4.index = 4;
    ugv4.name = "UGV4";
    ugv4.color = "goldenrod";
    ugv4.adHocRadius = 50;

    // initialize the base
    base1 = new Base();
    base1.x = .580 * uiMap.mapCanvas.width;
    base1.y = .987 * uiMap.mapCanvas.height;
    uiMap.mainBase = base1;

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

    // initialize the routers for the map traveling
    router1 = new Router();  // bottom left
    router1.x = .800 * uiMap.mapCanvas.width;
    router1.y = .850 * uiMap.mapCanvas.height;

    router2 = new Router();  // middle right
    router2.x = .800 * uiMap.mapCanvas.width;
    router2.y = .600 * uiMap.mapCanvas.height;

    router3 = new Router();  // top right
    router3.x = .850 * uiMap.mapCanvas.width;
    router3.y = .400 * uiMap.mapCanvas.height;

    router4 = new Router();  // top middle
    router4.x = .600 * uiMap.mapCanvas.width;
    router4.y = .300 * uiMap.mapCanvas.height;

    router5 = new Router();  // bottom middle left
    router5.x = .400 * uiMap.mapCanvas.width;
    router5.y = .850 * uiMap.mapCanvas.height;

    router6 = new Router();  // bottom left
    router6.x = .200 * uiMap.mapCanvas.width;
    router6.y = .750 * uiMap.mapCanvas.height;

    router7 = new Router();  // middle left
    router7.x = .200 * uiMap.mapCanvas.width;
    router7.y = .550 * uiMap.mapCanvas.height;

    router8 = new Router();  // top left
    router8.x = .150 * uiMap.mapCanvas.width;
    router8.y = .350 * uiMap.mapCanvas.height;
    
    
    // add them to the UI Map
    uiMap.uiObjects.push(cache1);
    uiMap.uiObjects.push(cache2);
    uiMap.uiObjects.push(cache3);
    uiMap.uiObjects.push(cache4);
    uiMap.uiObjects.push(cache5);

    uiMap.uiObjects.push(base1);

    uiMap.uiObjects.push(router1);
    uiMap.uiObjects.push(router2);
    uiMap.uiObjects.push(router3);
    uiMap.uiObjects.push(router4);
    uiMap.uiObjects.push(router5);
    uiMap.uiObjects.push(router6);
    uiMap.uiObjects.push(router7);
    uiMap.uiObjects.push(router8);

    uiMap.uiObjects.push(ugv1);
    uiMap.uiObjects.push(ugv2);
    uiMap.uiObjects.push(ugv3);
    uiMap.uiObjects.push(ugv4);
    

    // initialize the end conditions
    uiMap.endCheck = stage3EndCheck;

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment";

    var instructionsTop = document.getElementById("instructions-top");
    instructionsTop.classList = "instructions grey-instructions";
    instructionsTop.innerHTML = "Use the Ground robots to collect the caches and return them to the base! When you are close to a cache, the robot's \"Collect Cache\" button will turn colored. Click the button to pick up the cache and then return the robot to the base.";

    var instructionsLeft = document.getElementById("instructions-cache");
    instructionsLeft.style.display = "flex";
    instructionsLeft.innerHTML = "Pay attention to the vehicle cameras to keep your robots from crashing! If a robot with a cache gets stuck, you can use another robot to collect its cache.";

    document.getElementById("cam1_button").innerHTML = "Collect Cache";
    document.getElementById("cam2_button").innerHTML = "Collect Cache";
    document.getElementById("cam3_button").innerHTML = "Collect Cache";
    document.getElementById("cam4_button").innerHTML = "Collect Cache";

    document.getElementById("cam1_button").onclick = () => {
        collectCache(ugv1.x, ugv1.y, ugv1);
    }
    document.getElementById("cam2_button").onclick = () => {
        collectCache(ugv2.x, ugv2.y, ugv2);
    }
    document.getElementById("cam3_button").onclick = () => {
        collectCache(ugv3.x, ugv3.y, ugv3);
    }
    document.getElementById("cam4_button").onclick = () => {
        collectCache(ugv4.x, ugv4.y, ugv4);
    }
}

var cacheList = [false, false, false, false, false];

function collectCache(x, y, ugv) {
    console.log("collectCache button pressed for UGV position", x, y)
    log({"stage": uiMap.stage, "action": "collectCache button pressed", "vehicle": ugv.name});
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a cache or vehicle
        if (uiMap.uiObjects[i].constructor.name != "Cache" && uiMap.uiObjects[i].constructor.name != "Vehicle") {
            continue;
        }

        // ignore if itself
        if (uiMap.uiObjects[i] == ugv) {
            continue;
        }

        // get the distance
        let dist = distance([x, y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);
        dist_scale = .05;

        // if UAV is within this cache range
        //console.log("distance to cache", i, "is", distance([x, y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]), "==", dist_scale * uiMap.mapCanvas.width, cacheList[i]);
        if (uiMap.uiObjects[i].constructor.name == "Cache" && cacheList[i] == false && dist < dist_scale * uiMap.mapCanvas.width) {
            // if cache already taken from this area, continue
            if (cacheList[i] == true) {
                console.log("cache already collected");
                continue;
            }

            ugv.carryingCache = true;  // set the ugv to be carrying the cache
            ugv.carryingCacheId = uiMap.uiObjects[i].index;
            uiMap.uiObjects[i].hide = true;  // hide the cache that was picked up

            // mark the cache as collected
            uiMap.cacheStates[i] = 10 + (ugv.index-1)

            cacheList[i] = true;
            console.log("collected cache");
            log({"stage": uiMap.stage, "action": "cache collected", "cacheId": i});
        }

        else if (uiMap.uiObjects[i].constructor.name == "Vehicle" && dist < dist_scale * uiMap.mapCanvas.width) {
            if (uiMap.uiObjects[i].carryingCache) {
                uiMap.uiObjects[i].carryingCache = false;
                ugv.carryingCache = true;
                ugv.carryingCacheId = uiMap.uiObjects[i].carryingCacheId;  // transfer the carrying cache ID
                uiMap.uiObjects[i].carryingCacheId = -1;  // remove the carrying cache ID from the previous robot
                // mark the cache as transfered
                uiMap.cacheStates[ugv.carryingCacheId] = 10 + (ugv.index-1)
                console.log("transferred cache", uiMap.uiObjects[i].carryingCache, ugv.carryingCache);
                log({"stage": uiMap.stage, "action": "cache transferred", "this": ugv.name, "target": uiMap.uiObjects[i].name});
            }
        }
    }
}

function checkButtons() {
    ugv1_x = 0;
    ugv1_y = 0;
    ugv1_cc = false;
    
    ugv2_x = 0;
    ugv2_y = 0;
    ugv2_cc = false;

    ugv3_x = 0;
    ugv3_y = 0;
    ugv3_cc = false;

    ugv4_x = 0;
    ugv4_y = 0;
    ugv4_cc = false;

    // assign UGV positions
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        if (uiMap.uiObjects[i].name == "UGV1") {
            ugv1_x = uiMap.uiObjects[i].x;
            ugv1_y = uiMap.uiObjects[i].y;
            ugv1_cc = uiMap.uiObjects[i].carryingCache;
        }
        if (uiMap.uiObjects[i].name == "UGV2") {
            ugv2_x = uiMap.uiObjects[i].x;
            ugv2_y = uiMap.uiObjects[i].y;
            ugv2_cc = uiMap.uiObjects[i].carryingCache;
        }
        if (uiMap.uiObjects[i].name == "UGV3") {
            ugv3_x = uiMap.uiObjects[i].x;
            ugv3_y = uiMap.uiObjects[i].y;
            ugv3_cc = uiMap.uiObjects[i].carryingCache;
        }
        if (uiMap.uiObjects[i].name == "UGV4") {
            ugv4_x = uiMap.uiObjects[i].x;
            ugv4_y = uiMap.uiObjects[i].y;
            ugv4_cc = uiMap.uiObjects[i].carryingCache;
        }
    }

    // set colors to initial values
    let cam1_color = "darkgrey";
    let cam2_color = "darkgrey";
    let cam3_color = "darkgrey";
    let cam4_color = "darkgrey";
    
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a cache or vehicle
        if (uiMap.uiObjects[i].constructor.name != "Cache" && uiMap.uiObjects[i].constructor.name != "Vehicle") {
            continue;
        }

        // get the distance
        let dist_1 = distance([ugv1_x, ugv1_y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);
        let dist_2 = distance([ugv2_x, ugv2_y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);
        let dist_3 = distance([ugv3_x, ugv3_y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);
        let dist_4 = distance([ugv4_x, ugv4_y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]);
        dist_scale = .05;

        

        // if UGV1 is within this cache range        
        if (((uiMap.uiObjects[i].constructor.name == "Cache" && cacheList[i] == false) || (uiMap.uiObjects[i].constructor.name == "Vehicle" && uiMap.uiObjects[i].carryingCache && !ugv1_cc)) && dist_1 < dist_scale * uiMap.mapCanvas.width) {
            cam1_color = "lightpink";
        }

        // if UGV2 is within this cache range
        if (((uiMap.uiObjects[i].constructor.name == "Cache" && cacheList[i] == false) || (uiMap.uiObjects[i].constructor.name == "Vehicle" && uiMap.uiObjects[i].carryingCache && !ugv2_cc)) && dist_2 < dist_scale * uiMap.mapCanvas.width) {
            cam2_color = "plum";
        }

        // if UGV3 is within this cache range
        if (((uiMap.uiObjects[i].constructor.name == "Cache" && cacheList[i] == false) || (uiMap.uiObjects[i].constructor.name == "Vehicle" && uiMap.uiObjects[i].carryingCache && !ugv3_cc)) && dist_3 < dist_scale * uiMap.mapCanvas.width) {
            cam3_color = "lightblue";
        }

        // if UGV4 is within this cache range
        if (((uiMap.uiObjects[i].constructor.name == "Cache" && cacheList[i] == false) || (uiMap.uiObjects[i].constructor.name == "Vehicle" && uiMap.uiObjects[i].carryingCache && !ugv4_cc)) && dist_4 < dist_scale * uiMap.mapCanvas.width) {
            cam4_color = "lightyellow";
        }
    }

    document.getElementById("cam1_button").style.backgroundColor = cam1_color;
    document.getElementById("cam2_button").style.backgroundColor = cam2_color;
    document.getElementById("cam3_button").style.backgroundColor = cam3_color;
    document.getElementById("cam4_button").style.backgroundColor = cam4_color;
}


function stage3EndCheck() {
    // if all caches are connected, end
    if (uiMap.stage == 3 && uiMap.returnedCaches >= 5) {
        return true;
    }

    // check if any caches need to be dropped off at the base or if a robot is stuck
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a vehicle
        if (uiMap.uiObjects[i].constructor.name != "Vehicle") {
            continue;
        }
        
        // if UGV is within the base range, check if it can drop off a package
        if (uiMap.uiObjects[i].carryingCache && distance([base1.x, base1.y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < .08 * uiMap.mapCanvas.width) {
            uiMap.returnedCaches += 1;
            uiMap.uiObjects[i].carryingCache = false;
            uiMap.cacheStates[uiMap.uiObjects[i].carryingCacheId] = 2;
            
            log({"stage": uiMap.stage, "action": "cache returned", "cacheId": i});
            console.log("returned cache");
        }

        // if stuck popup has not happened yet and the timer has been 10 seconds, run checks
        if (!uiMap.announcedStuck && Date.now() / 1000 - uiMap.uiObjects[i].stuckTimer > 10) {
            uiMap.uiObjects[i].stuckTimer = Date.now() / 1000;
            // if robot has no waypoints, set stuck X/Y to -1 and move on
            if (uiMap.uiObjects[i].waypoints.length == 0) {
                //console.log("robot", i - 13, "reset stuck");
                uiMap.uiObjects[i].stuckX = -1;
                uiMap.uiObjects[i].stuckY = -1;
            }
            // else if the stuck x/y are close to the current, announce
            else if (!workerId.contains("replay") && Math.abs(uiMap.uiObjects[i].stuckX - uiMap.uiObjects[i].x) < 1.0 && Math.abs(uiMap.uiObjects[i].stuckY - uiMap.uiObjects[i].y) < 1.0) {
                let robot_color = uiMap.uiObjects[i].color;
                //console.log("robot", i - 13, "alerting stuck");
                alert("It appears the " + robot_color + " robot is stuck! You are unlikely to free it, so use the other robots to complete the mission.\n\nIf the stuck robot is carrying a cache, you can bring another robot close to it and press its \"Collect Cache\" button.")
                uiMap.announcedStuck = true;
            }
            else {
                //console.log("robot", i - 13, "set stuck to current");
                uiMap.uiObjects[i].stuckX = uiMap.uiObjects[i].x;
                uiMap.uiObjects[i].stuckY = uiMap.uiObjects[i].y;
            }

            if (workerId.contains("replay")) {
                uiMap.announcedStuck = true;
            }
        }
    }

    // check if any vehicles are out of range
    manageAdHocRobots();

    // check if a vehicle carrying a cache is close to base
    return false;
}
