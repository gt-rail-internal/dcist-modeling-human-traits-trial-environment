// Create the game objects for Stage 1
function initStage1() { 

    // set the uiMap to use the network (server)
    uiMap.networked = true;
    uiMap.stage = 1;


    // initialize two UAVs and two UGVs
    uav1 = new Vehicle("uav");
    uav1.index = 1;
    uav1.name = "UAV1";
    uav1.color = "red"

    uav2 = new Vehicle("uav");
    uav2.index = 2;
    uav2.name = "UAV2";
    uav2.color = "purple";
    

    uav3 = new Vehicle("uav");
    uav3.index = 3;
    uav3.name = "UAV3";
    uav3.color = "blue";

    uav4 = new Vehicle("uav");
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
    cacheArea1.cacheX = 174.75; // good, top left
    cacheArea1.cacheY = 200.02;
    
    cacheArea2 = new CacheArea();
    cacheArea2.x = (.564 - .01) * uiMap.mapCanvas.width;
    cacheArea2.y = (.327 + .06) * uiMap.mapCanvas.height; 
    cacheArea2.cacheX = 400.39;  // good, top center
    cacheArea2.cacheY = 221.38;

    cacheArea3 = new CacheArea();
    cacheArea3.x = (.900 - .01) * uiMap.mapCanvas.width;
    cacheArea3.y = (.374 - .04) * uiMap.mapCanvas.height; 
    cacheArea3.cacheX = 627.15;  // good, top right
    cacheArea3.cacheY = 254.40;

    cacheArea4 = new CacheArea();
    cacheArea4.x = (.895 - .05) * uiMap.mapCanvas.width;
    cacheArea4.y = (.877 - 0) * uiMap.mapCanvas.height; 
    cacheArea4.cacheX = 633.04;  // good, bottom right
    cacheArea4.cacheY = 621.88;

    cacheArea5 = new CacheArea();
    cacheArea5.x = (.078 + .03) * uiMap.mapCanvas.width;
    cacheArea5.y = (.758 - .02) * uiMap.mapCanvas.height; 
    cacheArea5.cacheX = 81.93; // good, bottom left
    cacheArea5.cacheY = 514.03;
    
    // add them to the UI Map
    uiMap.uiObjects.push(cacheArea1);
    uiMap.uiObjects.push(cacheArea2);
    uiMap.uiObjects.push(cacheArea3);
    uiMap.uiObjects.push(cacheArea4);
    uiMap.uiObjects.push(cacheArea5);

    uiMap.uiObjects.push(base1);
    uiMap.uiObjects.push(uav1);
    uiMap.uiObjects.push(uav2);
    uiMap.uiObjects.push(uav3);
    uiMap.uiObjects.push(uav4);
    

    // initialize the end conditions
    uiMap.endCheck = stage1EndCheck;
    uiMap.displayAdHocRanges = false;

    var title = document.getElementById("titlebar");
    title.innerHTML = "Simulation Environment - Stage 1";

    var instructionsTop = document.getElementById("instructions-top");
    instructionsTop.classList = "instructions grey-instructions";
    instructionsTop.innerHTML = "Search the grey areas to find five supply caches. Each grey area has one cache. When a cache is found, mark it! You can expect low framerates with the aerial robot cameras and position updates.";

    document.getElementById("instructions-cache").style.display = "flex";

    document.getElementById("cam1_button").innerHTML = "Mark Cache";
    document.getElementById("cam2_button").innerHTML = "Mark Cache";
    document.getElementById("cam3_button").innerHTML = "Mark Cache";
    document.getElementById("cam4_button").innerHTML = "Mark Cache";

    document.getElementById("cam1_button").onclick = () => {
        addCache(uav1.x, uav1.y);
    }
    document.getElementById("cam2_button").onclick = () => {
        addCache(uav2.x, uav2.y);
    }
    document.getElementById("cam3_button").onclick = () => {
        addCache(uav3.x, uav3.y);
    }
    document.getElementById("cam4_button").onclick = () => {
        addCache(uav4.x, uav4.y);
    }
}

var cacheList = [false, false, false, false, false];

function addCache(x, y) {
    console.log("addCache button pressed for UAV position", x, y)
    for (let i=0; i<uiMap.uiObjects.length; i++) {
        // ignore if not a cache area
        if (uiMap.uiObjects[i].constructor.name != "CacheArea") {
            continue;
        }

        // if UAV is within this cache range
        console.log("distance to cache", i, "is", distance([x, y], [uiMap.uiObjects[i].cacheX, uiMap.uiObjects[i].cacheY]), "==", .05 * uiMap.mapCanvas.width);
        if (distance([x, y], [uiMap.uiObjects[i].cacheX, uiMap.uiObjects[i].cacheY]) < .05 * uiMap.mapCanvas.width) {
            // if cache already exists in this area, continue
            if (cacheList[i] == true) {
                console.log("cache already exists");
                continue;
            }

            // otherwise make the cache (will later add a distance to actual cache location check)
            cache = new Cache();
            cache.name = "Cache " + String(i + 1);
            cache.x = x
            cache.y = y
            uiMap.uiObjects.splice(5, 0, cache);
            cacheList[i] = true;

            uiMap.knownCaches += 1;  // variable to count the number of known caches

            console.log("created cache");
        }
    }
}


function stage1EndCheck() {
    // if all caches are connected, end
    if (uiMap.stage == 1 && uiMap.knownCaches >= 5) {
        return true;
    }
    return false;
}
