// utility.js contains several functions for calculations, mouse handling, and logging

// simple logging function, sends the given data to our webserver, you will probably want to change this for your own purposes as it is set up for our RAIL@GT experiment
function log(data) {
    data["worker-id"] = workerId;  // include the worker ID
    return fetch("/logging", {method: "POST", body: JSON.stringify(data)});
}

// euclidean distance function, used for determining whether a mouse click is close to a node on the gameboard
function distance(a, b) {
    return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
}

// handles mouse clicks to select nodes
function mouseHandler(event) {
    // if the user clicked on the gameboard canvas
    if (event.originalTarget == document.getElementById("Canvas") || event.target == document.getElementById("Canvas")) {
        // get the X/Y coordinates of the canvas click
        var canvasX = event.clientX - (event.originalTarget ? event.originalTarget.offsetLeft : event.target.offsetLeft) + window.scrollX;
        var canvasY = event.clientY - (event.originalTarget ? event.originalTarget.offsetTop : event.target.offsetTop) + window.scrollY;
        var canvasClick = [canvasX, canvasY];

        // for each ball, check if the click was close
        for (let i=0; i<gameboard.balls.length; i++) {
            // get the X/Y coordinates of the ball
            ballX = gameboard.balls[i].x;
            ballY = gameboard.balls[i].y;

            // check if the click was close to the node
            if (distance(canvasClick, [ballX, ballY]) < gameboard.balls[i].height * .6) {
                // if the gameboard is waiting for a user to click, set the selected node, otherwise ignore
                if (gameboard.state == "response") {
                    // set game variables to account for the ball being selected
                    let oldSelection = gameboard.balls[i].selected;
                    gameboard.balls[i].selected = gameboard.numSelected < gameboard.tracking ? !gameboard.balls[i].selected : false;
                    // if the ball went from unselected to selected, increase the number selected
                    if (!oldSelection && gameboard.balls[i].selected) {
                        gameboard.numSelected += 1;
                    }
                    // if the ball went from selected to unselected, decrease the number selected
                    if (oldSelection && !gameboard.balls[i].selected) {
                        gameboard.numSelected -= 1;
                    }
                    // if the number selected is the max, display the next button
                    if (gameboard.numSelected == gameboard.tracking) {
                        document.getElementById("next-button").style.display = "block";
                    }
                    else {
                        document.getElementById("next-button").style.display = "none";
                    }
                    // log which ball was clicked
                    log({"stage": "object tracking", "action": "ball selected", "object": i});
                }
                // stop checking nodes, since we know which one the user clicked
                break;
            }
        }
    }
}

// gets the difference between angles, in radians
function getAngleDiff(angle1, angle2) {
    let diff = angle1 - angle2;
    if (diff < 0) {
        diff = diff + 2 * Math.PI;
    }
    if (diff >= 2 * Math.PI) {
        diff = diff - 2 * Math.PI;
    }
    return diff;
}

// fades all the selected balls from blue to red
function fadeBalls(fadeStart) {
    // get the seconds elapsed on the fade (0 to 5)
    fadeStep = (Date.now() - fadeStart) / 1000;

    // clamp the fade step at 5 seconds
    if (fadeStep > 5) {
        fadeStep = 5;
    }

    // determine the current color
    let color = "rgb(" + (fadeStep * 255 / 5) + ", 0, " + (255 - fadeStep * 255 / 5) + ")";
    
    // assign the tracked balls the color
    for (let i=0; i<gameboard.balls.length; i++) {
        if (gameboard.balls[i].tracked) {
            gameboard.balls[i].color = color;
        }
    }

    // if the fade isn't complete, loop
    if (fadeStep < 5) {
        setTimeout(() => { fadeBalls(fadeStart)} , 30);
    }

    return
}

// implementation of MurmerHash3 mixing function, taken from (https://stackoverflow.com/a/47593316/3427093)
function xmur3(str) {
    for(var i = 0, h = 1779033703 ^ str.length; i < str.length; i++)
        h = Math.imul(h ^ str.charCodeAt(i), 3432918353),
        h = h << 13 | h >>> 19;
    return function() {
        h = Math.imul(h ^ h >>> 16, 2246822507);
        h = Math.imul(h ^ h >>> 13, 3266489909);
        return (h ^= h >>> 16) >>> 0;
    }
}

// implementation of Mulberry 32-bit pseudorandom number generator, taken from (https://stackoverflow.com/a/47593316/3427093)
function mulberry32(a) {
    return function() {
      var t = a += 0x6D2B79F5;
      t = Math.imul(t ^ t >>> 15, t | 1);
      t ^= t + Math.imul(t ^ t >>> 7, t | 61);
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    }
}