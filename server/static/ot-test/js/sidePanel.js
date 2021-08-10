// sidePanel.js contains functions that define the side panel content at various parts of the minigame
// you can change the content at will, such as by adding an <input> box for a user to enter their ID in the sidePanelIntroduction()

// side panel for the game introduction
function sidePanelIntroduction() {
    // log that the user is seeing this panel
    log({"stage": "object tracking", "action": "side-panel-state", "object": "showing introduction"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Introduction</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Welcome! In this game you will be tracking balls as they move across the stage. You will play " + targetSteps.length + " levels of increasing difficulty. Do your best!";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>You will be scored by the number of balls you successfully track in each round. If you are unable to track all the balls, track as many as you can.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>Click \"Next\" to continue.";
    var text4 = document.createElement("div"); text4.innerHTML = "";
    var text5 = document.createElement("div"); text5.innerHTML = "";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);
    content.appendChild(text4);
    content.appendChild(text5);

    document.getElementById("next-button").style.display = "block";
    document.getElementById("next-button").innerHTML = "Next";

    // when the "Next" button is clicked, display the observation side panel
    document.getElementById("next-button").onclick = function () {
        log({"stage": "object tracking", "action": "side panel click", "object": "next button introduction"});
        sidePanelObservation();
        gameboard.nextGame();
    };
}

// side panel for the user observation state
function sidePanelObservation() {
    // log that the user is seeing this panel
    log({"stage": "object tracking", "action": "side-panel-state", "object": "showing tutorial panel 1"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Observation</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#E8E8E8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Track the blue ball" + (gameboard.tracking > 1 ? "s" : "") + " as " + (gameboard.tracking > 1 ? "they" : "it") + " turn" + (gameboard.tracking > 1 ? "" : "s") + " red and bounce" + (gameboard.tracking > 1 ? "" : "s") + " around the stage.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>After " + gameboard.freezeTimer + " seconds you will be asked to identify which ball" + (gameboard.tracking > 1 ? "s" : "") + " you are tracking.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>To start, press \"Begin\".";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);

    nextButton = document.getElementById("next-button");

    nextButton.style.display = "block";
    nextButton.innerHTML = "Begin";

    // when the "Begin" button is clicked, start the propagation (in the observation state)
    nextButton.onclick = function () {
        log({"stage": "object tracking", "action": "side-panel-click", "object": "begin tracking"});
        gameboard.state = "observation";
        nextButton.style.display = "none";
        gameboard.start();
        fadeBalls(Date.now());
    };
}

// side panel for the user response state
function sidePanelResponse() {
    // log that the user is seeing this panel
    log({"stage": "object tracking", "action": "side-panel-state", "object": "showing response panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Your Turn</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "#F8E8F8";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Now, select the ball" + (gameboard.tracking > 1 ? "s" : "") + " that you were tracking.";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Click on a ball to select it. To deselect it, click it again.";
    var text3 = document.createElement("div"); text3.innerHTML = "<br>You must select " + gameboard.tracking + " ball" + (gameboard.tracking > 1 ? "s" : "") + " before continuing.";

    content.appendChild(text1);
    content.appendChild(text2);
    content.appendChild(text3);

    // hide the resume button if game active
    nextButton = document.getElementById("next-button");
    nextButton.style.display = "none";
    nextButton.innerHTML = "Done";
    nextButton.onclick = function() {
        log({"stage": "object tracking", "action": "side-panel-click", "object": "side panel clicked done button"});
        gameboard.scoreRound();
        sidePanelObservation();
        gameboard.nextGame();
    }
}

// side panel for the puzzle is complete
function sidePanelPuzzleComplete() {
    // log that the user is seeing this panel
    log({"stage": "object tracking", "action": "side-panel-state", "object": "showing puzzle complete panel"});

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

    document.getElementById("object tracking next-button").style.display = "block";
    document.getElementById("object tracking next-button").innerHTML = "Continue";

    // when the "Continue" button is pressed, move on to the next puzle
    document.getElementById("object tracking next-button").onclick = function() {
        log({"stage": "object tracking", "action": "side-panel-click", "object": "next button puzzle complete"});
        gameboard.nextGame();
    };

    log({"stage": "object tracking", "action": "side-panel-state", "object": "round complete"});
}

// side panel for the game is over
function sidePanelEndGame() {
    // log that the user is seeing this panel
    log({"stage": "object tracking", "action": "side-panel-state", "object": "showing endgame panel"});

    var title = document.getElementById("SidePanelTitle");
    title.innerHTML = "<b>Game Complete</b>";

    var content = document.getElementById("SidePanelContent");
    content.innerHTML = "";

    document.getElementById("SidePanel").style.backgroundColor = "lightyellow";

    var text1 = document.createElement("div"); text1.innerHTML = "<br>Congratulations! You completed this minigame!";
    var text2 = document.createElement("div"); text2.innerHTML = "<br>Please notify the researcher.";

    // after 5 seconds, redirect the user to the experiment portal
    //window.setTimeout(() => {
    //    window.location.href = "portal?pageFrom=1&success=1&workerId=" + workerId + "&mission=" + mission;
    //}, 5000);

    content.appendChild(text1);
    content.appendChild(text2);

    if (showScore){
        var text3 = document.createElement("div"); text3.innerHTML = "<br>Score (correct balls, per round):<br>" + gameboard.scores + "<br><br>Please notify the overseer that you have completed this stage.";
        content.appendChild(text3);
    }
    log({"stage": "object tracking", "result": gameboard.scores})

    document.getElementById("next-button").style.display = "none";
}