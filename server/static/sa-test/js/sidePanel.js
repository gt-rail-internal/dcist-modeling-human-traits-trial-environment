// sidePanel.js: functions used to fill in the right side panel at various test stages

function sidePanelTutorial1() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing tutorial panel 1"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Tutorial</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>These are warehouses. Warehouses have a shape and a color. You will be watching packages sent through a warehouse network for processing.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>When a package arrives at a warehouse, it will be processed if the shape and color match. Processed packages will disappear. Otherwise, the package will be stored in the bar to the right of the warehouse and sent to a downstream warehouse.";
    var text3 = document.createElement("div"); text3.innerHTML = "";//"<br>Warehouses will try to forward packages to the best fit warehouse: first priority is by matching shape and color, then shape, then color, and lastly randomly."
    var text4 = document.createElement("div"); text4.innerHTML = "<br>If a warehouse is at maximum capacity, it will be unable to receive packages. Click \"Play\" to see this demo simulation.";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);

    document.getElementById("sagat-resume").style.display = "block";
    document.getElementById("sagat-resume").innerHTML = "Play";
    document.getElementById("sagat-resume").onclick = function () {  log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button tutorial 1"}); sidePanelTutorial2(); gameActive = true; };
}

function sidePanelTutorial2() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing tutorial panel 2"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Tutorial</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Take a minute to watch the simulation, to gain an understanding of how packages are stored in warehouses, how packages are forwarded, and what happens when a warehouse is full.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>In 30 seconds you will be able to continue.";

    content.appendChild(text1);
    content.appendChild(text2);

    document.getElementById("sagat-resume").style.display = "none";

    window.setTimeout(() => {
        document.getElementById("sagat-resume").style.display = "block";
        document.getElementById("sagat-resume").innerHTML = "Continue";
        document.getElementById("sagat-resume").onclick = function () {
            log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button tutorial 2"});
            gameActive = true;
            sagat.freeze();
            sidePanelTutorial3();
        };
    }, 30000)
    
}

function sidePanelTutorial3() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing tutorial panel 3"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Tutorial</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>After watching the simulation for 30 seconds, you will be asked to color the warehouses based on how full they are. Try it now, by clicking the warehouses to cycle their colors to green, red, and orange.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>You will be able to continue in 15 seconds.";

    content.appendChild(text1);
    content.appendChild(text2);

    document.getElementById("sagat-resume").style.display = "none";

    window.setTimeout(() => {
        document.getElementById("sagat-resume").style.display = "block";
        document.getElementById("sagat-resume").innerHTML = "Continue";
        document.getElementById("sagat-resume").onclick = function () {
            log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button tutorial 3"});
            gameboard = new Gameboard(context);
            gameboard.generateGameboard();
            gameActive = false;
            tutorialActive = false;
            sagat = new SAGAT(30);
            sidePanelShowIntro();
        };
    }, 15000)
}

function sidePanelShowIntro() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing introduction"});

    // reset the random number generator
    seed = xmur3("RAIL Lab");  // use xmur3 to create a seed hash
    rand = mulberry32(seed());  // use the mulberry32 algorithm for the random number generator

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Introduction</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Now for the actual game! Welcome!";
    var text2 = document.createElement("div"); text2.innerHTML = "";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>You will watch the warehouse simulation on the left for 30 seconds, and then you be asked to mark warehouses as full, nearing capacity, or not nearing capacity. Use the capacity bars next to each warehouse to guide your understanding.";
    var text4 = document.createElement("div"); text4.innerHTML = "<br>Trying to memorize the state of every warehouse might be difficult, so instead focus on understanding the network overall.";
    var text5 = document.createElement("div"); text5.innerHTML = "<br>When you are ready, click \"Continue\" to review the instructions.";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);

    document.getElementById("sagat-resume").style.display = "block";
    document.getElementById("sagat-resume").innerHTML = "Continue";
    document.getElementById("sagat-resume").onclick = function () { log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button introduction"}); sidePanelShowInformation(); };
}


function sidePanelShowInformation() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing information panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Instructions</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>You are the manager for this warehouse distribution network!";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Warehouses only accept packages of the correct shape and color.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>When a warehouse receives unwanted packages, it stores and forwards them to downstream warehouses. When a warehouse is full of unwanted packages, it will not accept any more packages.";
    var text4 = document.createElement("div"); text4.innerHTML = ""; //"<br>When forwarding a package, warehouses prioritize sending to the correct shape/color, then the correct shape, then the correct color, and lastly randomly.";
    var text5 = document.createElement("div"); text5.innerHTML = "<br>Do your best to stay aware of the warehouse capacities."

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);

    // hide the resume button if game active
    document.getElementById("sagat-resume").style.display = gameActive ? "none" : "block";
    document.getElementById("sagat-resume").innerHTML = "Begin!";
    document.getElementById("sagat-resume").onclick = function() {
        log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button information"});
        sagat.start();
        sagat.isRunning = true;
        gameActive = true;
        sidePanelShowInformation();
    };
}

function sidePanelSAGATQuestions() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing sagat questions"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Questions</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "aliceblue";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Now, we need you to access the health of the network.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Clicking a warehouse will cycle its color from green, to red, to orange.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>Which warehouses are at capacity (>90%)? Color them red.";
    var text4 = document.createElement("div"); text4.innerHTML = "<br>Which warehouses are nearing capacity (>60%)? Color them orange.";
    var text5 = document.createElement("div"); text5.innerHTML = "<br>Which warehouses are not close to capacity? Color them green.";
    var text6 = document.createElement("div"); text6.innerHTML = "<br><b><u>You will need to color in all the warehouses</u></b>. When you have completed, click \"Resume\".";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);
    content.appendChild(text6);

    document.getElementById("sagat-resume").style.display = "block";
    document.getElementById("sagat-resume").innerHTML = "Resume";

    document.getElementById("sagat-resume").onclick = function() { sagat.unfreeze(); }
}


function sidePanelIncompleteSelection() {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing incomplete cycle panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Completeness Check</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "mistyrose";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>It seems there is a misunderstanding as to which warehouses to color.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>You should color all warehouses to the best of your ability, leaving none colored grey.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>Click \"Resume\" to color the warehouses again.";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);

    document.getElementById("sagat-resume").style.display = "block";
    document.getElementById("sagat-resume").innerHTML = "Resume";
    document.getElementById("sagat-resume").onclick = function() { log({"stage": "SAGAT", "action": "side-panel-click", "object": "next button incomplete panel"}); sidePanelSAGATQuestions(); };
}


function sidePanelEndGame(scores) {
    log({"stage": "SAGAT", "action": "side-panel-state", "object": "showing endgame panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Results</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "lightyellow";

    let score = scores.reduce((a, b) => a + b, 0);

    log({"stage": "SAGAT", "action": "complete", "score": score});

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Congratulations! You completed this minigame!";
    var text2 = document.createElement("div"); text2.innerHTML = ""; //<br>Your score is: " + (score / (15 * scores.length)).toFixed(2);
    var text3 = document.createElement("div"); text3.innerHTML = "<br>Please tell the researcher that you have completed this stage.";
    var text4 = document.createElement("div"); text4.innerHTML = "";
    var text5 = document.createElement("div"); text5.innerHTML = "";
    /*text5.style.display = "flex";
    text5.style.justifyContent = "center";

    text5.onclick = () => {
        window.location.href = "portal?pageFrom=1&success=1&workerId=" + workerId + "&mission=" + mission;
    }

    window.setTimeout(() => {
        window.location.href = "portal?pageFrom=1&success=1&workerId=" + workerId + "&mission=" + mission;
    }, 5000);*/


    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);

    document.getElementById("sagat-resume").style.display = "none";   
}
