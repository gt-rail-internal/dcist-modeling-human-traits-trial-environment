// Create the game objects for Stage 1
function initStage1() { 

    // set the uiMap to use the network (server)
    uiMap.networked = true;
    uiMap.stage = 2;


    // initialize two UAVs and two UGVs
    uav1 = new Vehicle("uav");
    uav1.initPositionListener("uav1");
    uav1.index = 1;
    uav1.name = "UAV1";
    uav1.color = "red"

    uav2 = new Vehicle("uav");
    uav2.initPositionListener("uav2");
    uav2.index = 2;
    uav2.name = "UAV2";
    uav2.color = "purple";
    

    uav3 = new Vehicle("uav");
    uav3.initPositionListener("uav3");
    uav3.index = 3;
    uav3.name = "UAV3";
    uav3.color = "blue";

    uav4 = new Vehicle("uav");
    uav4.initPositionListener("uav4");
    uav4.index = 4;
    uav4.name = "UAV4";
    uav4.color = "goldenrod"

    // initialize the base
    base1 = new Base();
    base1.x = .580 * uiMap.mapCanvas.width;
    base1.y = .987 * uiMap.mapCanvas.height;

    cacheArea1 = new CacheArea();
    cacheArea1.x = (.200 + .03) * uiMap.mapCanvas.width;
    cacheArea1.y = (.298 + .07) * uiMap.mapCanvas.height; 
    
    cacheArea2 = new CacheArea();
    cacheArea2.x = (.564 - .01) * uiMap.mapCanvas.width;
    cacheArea2.y = (.327 + .06) * uiMap.mapCanvas.height; 

    cacheArea3 = new CacheArea();
    cacheArea3.x = (.900 - .01) * uiMap.mapCanvas.width;
    cacheArea3.y = (.374 - .04) * uiMap.mapCanvas.height; 

    cacheArea4 = new CacheArea();
    cacheArea4.x = (.895 - .05) * uiMap.mapCanvas.width;
    cacheArea4.y = (.877 - 0) * uiMap.mapCanvas.height; 

    cacheArea5 = new CacheArea();
    cacheArea5.x = (.078 + .03) * uiMap.mapCanvas.width;
    cacheArea5.y = (.758 - .02) * uiMap.mapCanvas.height; 
    
    // add them to the UI Map
    uiMap.uiObjects.push(uav1);
    uiMap.uiObjects.push(uav2);
    uiMap.uiObjects.push(uav3);
    uiMap.uiObjects.push(uav4);
    uiMap.uiObjects.push(base1);

    uiMap.uiObjects.push(cacheArea1);
    uiMap.uiObjects.push(cacheArea2);
    uiMap.uiObjects.push(cacheArea3);
    uiMap.uiObjects.push(cacheArea4);
    uiMap.uiObjects.push(cacheArea5);

    // initialize the end conditions
    uiMap.endCheck = stage1EndCheck;
    uiMap.displayAdHocRanges = false;
}


function stage1EndCheck() {
    // if all caches are connected, end
    if (uiMap.stage == 1 && uiMap.knownCaches.length >= 5) {
        return true;
    }
    return false;
}
