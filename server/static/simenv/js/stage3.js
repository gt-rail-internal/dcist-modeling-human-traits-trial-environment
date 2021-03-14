// Create the game objects for Stage 1
function initStage3() { 

    // set the uiMap to use the network (server)
    uiMap.networked = true;
    uiMap.stage = 3;
    uiMap.displayAdHocRanges = true;
    uiMap.adHocLock = true;

    // initialize two UAVs and two UGVs
    ugv1 = new Vehicle("ugv");
    ugv1.index = 1;
    ugv1.name = "UGV1";
    ugv1.color = "red";

    ugv2 = new Vehicle("ugv");
    ugv2.index = 2;
    ugv2.name = "UGV2";
    ugv2.color = "purple";
    

    ugv3 = new Vehicle("ugv");
    ugv3.index = 3;
    ugv3.name = "UGV3";
    ugv3.color = "blue";

    ugv4 = new Vehicle("ugv");
    ugv4.index = 4;
    ugv4.name = "UGV4";
    ugv4.color = "goldenrod"

    // initialize the base
    var base1 = new Base();
    base1.x = .580 * uiMap.mapCanvas.width;
    base1.y = .987 * uiMap.mapCanvas.height;

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
    router1 = new Router();
    router1.x = .500 * uiMap.mapCanvas.width;
    router1.y = .500 * uiMap.mapCanvas.height;

    router2 = new Router();
    router2.x = .500 * uiMap.mapCanvas.width;
    router2.y = .200 * uiMap.mapCanvas.height;
    
    
    // add them to the UI Map
    uiMap.uiObjects.push(cache1);
    uiMap.uiObjects.push(cache2);
    uiMap.uiObjects.push(cache3);
    uiMap.uiObjects.push(cache4);
    uiMap.uiObjects.push(cache5);

    uiMap.uiObjects.push(base1);

    uiMap.uiObjects.push(router1);
    uiMap.uiObjects.push(router2);

    uiMap.uiObjects.push(ugv1);
    uiMap.uiObjects.push(ugv2);
    uiMap.uiObjects.push(ugv3);
    uiMap.uiObjects.push(ugv4);
    

    // initialize the end conditions
    uiMap.endCheck = stage3EndCheck;
    uiMap.displayAdHocRanges = false;

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment - Stage 3";

    var instructionsTop = document.getElementById("instructions-top");
    instructionsTop.classList = "instructions grey-instructions";
    instructionsTop.innerHTML = "Use the Ground vehicles to collect the caches and return them to the base! When you are close to a cache, click the robot's \"Collect Cache\" button. You can expect low framerates with the ground robot cameras and position updates.";

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
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a cache area
        if (uiMap.uiObjects[i].constructor.name != "Cache") {
            continue;
        }

        // if UAV is within this cache range
        console.log("distance to cache", i, "is", distance([x, y], [uiMap.uiObjects[i].cacheX, uiMap.uiObjects[i].cacheY]), "==", .05 * uiMap.mapCanvas.width);
        if (distance([x, y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < .05 * uiMap.mapCanvas.width) {
            // if cache already taken from this area, continue
            if (cacheList[i] == true) {
                console.log("cache already collected");
                continue;
            }

            ugv.carryingCache = true;  // set the ugv to be carrying the cache
            uiMap.uiObjects[i].hide = true;  // hide the cache that was picked up

            cacheList[i] = true;
            console.log("collected cache");
        }
    }
}


function stage3EndCheck() {
    // if all caches are connected, end
    if (uiMap.stage == 3 && uiMap.returnedCaches >= 5) {
        return true;
    }

    // check if any caches need to be dropped off at the base
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a vehicle
        if (uiMap.uiObjects[i].constructor.name != "Vehicle") {
            continue;
        }
        
        // if UGV is within the base range
        console.log("distance to base is", distance([base1.x, base1.y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]), "==", .05 * uiMap.mapCanvas.width);
        if (uiMap.uiObjects[i].carryingCache && distance([base1.x, base1.y], [uiMap.uiObjects[i].x, uiMap.uiObjects[i].y]) < .05 * uiMap.mapCanvas.width) {
            uiMap.returnedCaches += 1;
            uiMap.uiObjects[i].carryingCache = false;
            console.log("returned cache");
        }
    }

    // check if a vehicle carrying a cache is close to base
    return false;
}
