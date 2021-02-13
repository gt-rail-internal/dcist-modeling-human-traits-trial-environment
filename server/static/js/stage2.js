// Create the game objects for Stage 1
function initStage2() { 
    // no cameras in this stage, so hide the camera feeds
    document.getElementById("left-panel").style.display = "none";
    document.getElementById("right-panel").style.display = "none";

    // initialize two UAVs and two UGVs
    uav1 = new Vehicle("uav");
    uav1.initPositionListener("uav1");
    uav1.index = 1;
    uav1.name = "UAV 1";
    uav1.color = "red"
    uav1.x = 10;
    uav1.y = 30;

    uav2 = new Vehicle("uav");
    uav2.initPositionListener("uav2");
    uav2.index = 2;
    uav2.name = "UAV 2";
    uav2.color = "purple";
    uav2.x = 150;
    uav2.y = 90;
    

    ugv1 = new Vehicle("ugv");
    ugv1.initPositionListener("ugv1");
    ugv1.index = 3;
    ugv1.name = "UGV 1";
    ugv1.color = "blue";
    ugv1.x = 20;
    ugv1.y = 160;

    ugv2 = new Vehicle("ugv");
    ugv2.initPositionListener("ugv2");
    ugv2.index = 4;
    ugv2.name = "UGV 2";
    ugv2.color = "goldenrod"
    ugv2.x = 80;
    ugv2.y = 40;

    // initialize five caches
    cache1 = new 
    

    // add them to the UI Map
    uiMap.uiObjects.push(uav1);
    uiMap.uiObjects.push(uav2);
    uiMap.uiObjects.push(ugv1);
    uiMap.uiObjects.push(ugv2);

}