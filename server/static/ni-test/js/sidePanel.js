// sidePanel.js contains functions that define the side panel content at various parts of the minigame
// you can change the content at will, such as by adding an <input> box for a user to enter their ID in the sidePanelIntroduction()

// side panel for the game introduction
function sidePanelIntroduction() {
    // log that the user is seeing this panel
    log({"stage": "networks", "action": "side-panel-state", "object": "showing introduction"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Introduction</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Welcome! In this game you will figure out how squares are connected in a set of " + networks.puzzles.length + " puzzles.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>A hidden network connects all the squares. When a square is selected, the squares connected to it will flip colors. Then the squares connected to those squares will flip. And so on until the entire network has flipped.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>You will first watch this happen for several different starting squares, so you can learn the hidden network.";
    var text4 = document.createElement("div"); text4.innerHTML = "<br>Then you will choose the square that flips the network in the least steps.";
    var text5 = document.createElement("div"); text5.innerHTML = "<br>When you are ready, click \"Next\" below.";
    
    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);

    document.getElementById("networks-next-button").style.display = "block";

    // if we collect the ID from the user, set that up here
    if (gameboard.askId) {
        text5.innerHTML = "<br>When you are ready, enter your Mechanical Turk ID below:";
        var workerIdInput = document.createElement("input"); workerIdInput.type = "text"; workerIdInput.id = "worker-id-input";
        content.appendChild(workerIdInput);
        workerIdInput.oninput = () => {
            document.getElementById("networks-next-button").style.display = "block";
            workerId = workerIdInput.value;
        };
        document.getElementById("networks-next-button").style.display = "none";
    }

    
    document.getElementById("networks-next-button").innerHTML = "Next";

    
    // when the "Next" button is clicked, display the observation side panel
    document.getElementById("networks-next-button").onclick = function () {
        log({"stage": "networks", "action": "side-panel-click", "object": "next button introduction"});
        sidePanelObservation();
    };
}

// side panel for the user observation state
function sidePanelObservation() {
    // log that the user is seeing this panel
    log({"stage": "networks", "action": "side-panel-state", "object": "showing observation panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Observation</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Watch closely as the network flips with several different starting squares.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Do your best to understand how the squares are connected.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>To start, press \"Begin\".";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);

    beginButton = document.getElementById("networks-next-button");

    beginButton.style.display = "block";
    beginButton.innerHTML = "Begin";

    // when the "Begin" button is clicked, start the propagation (in the observation state)
    beginButton.onclick = function () {
        log({"stage": "networks", "action": "side-panel-click", "object": "begin networks"});
        beginButton.style.display = "none";
        networks.start();
    };
}

// side panel for the user response state
function sidePanelResponse() {
    // log that the user is seeing this panel
    log({"stage": "networks", "action": "side-panel-state", "object": "showing response panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Your Turn</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#F8E8F8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Now, select a starting square for the network.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Choose the square that will turn the entire network blue in the least number of steps.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>You will have up to " + networks.maxRounds + " guesses."

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);

    // hide the resume button if game active
    document.getElementById("networks-next-button").style.display = "none";
}

// side panel for the puzzle is complete
function sidePanelPuzzleComplete() {
    // log that the user is seeing this panel
    log({"stage": "networks", "action": "side-panel-state", "object": "showing puzzle complete panel"});

    var title = document.getElementById("SidePanelTitle");
    var titleText = "<b>Puzzle Complete</b>";

    // if the side panel is already active, return
    if (title.innerHTML == titleText) {
        return;
    }
    else {
        title.innerHTML = titleText;
    }

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "aliceblue";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Well done, you have completed this puzzle!";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Click \"Continue\" below to move to the next puzzle.";

    content.appendChild(text1);
    content.appendChild(text2);

    document.getElementById("networks-next-button").style.display = "block";
    document.getElementById("networks-next-button").innerHTML = "Continue";

    // when the "Continue" button is pressed, move on to the next puzle
    document.getElementById("networks-next-button").onclick = function() {
        log({"stage": "networks", "action": "side-panel-click", "object": "next button puzzle complete"});
        networks.nextPuzzle();
    };

    log({"stage": "networks", "action": "side-panel-state", "object": "round complete"});
}

// side panel for the game is over
function sidePanelEndGame(score) {
    // log that the user is seeing this panel
    log({"stage": "networks", "action": "side-panel-state", "object": "showing endgame panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Game Complete</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "lightyellow";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Congratulations! You completed this minigame!";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Please notify the researcher that you have completed this stage.";

    // after 5 seconds, redirect the user to the experiment portal
    window.setTimeout(() => {
        //window.location.href = "http://4fe162699fc1.ngrok.io/tutorial?nextStage=4&mission=2" + "&workerId=" + workerId;
    }, 10000);

    content.appendChild(text1);
    content.appendChild(text2);

    if (networks.showScore){
        var text3 = document.createElement("div"); text3.innerHTML = "<br>Score: " + score;
        content.appendChild(text3);
    }

    document.getElementById("networks-next-button").style.display = "none";   
}
