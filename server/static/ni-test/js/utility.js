// utility.js contains several functions for calculations, mouse handling, and logging

// simple logging function, sends the given data to our webserver, you will probably want to change this for your own purposes as it is set up for our RAIL@GT experiment
function log(data) {
    data["worker-id"] = workerId;  // include the worker ID
    console.log("LOG", data);
    
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

        // for each node, check if the click was close
        for (i in networks.nodes) {
            // get the X/Y coordinates of the node
            nodeX = networks.nodes[i].location[0] * networks.nodes[i].context.canvas.width;
            nodeY = (1-networks.nodes[i].location[1]) * networks.nodes[i].context.canvas.height;

            // check if the click was close to the node
            if (distance(canvasClick, [nodeX, nodeY]) < networks.nodes[i].height * .75) {
                // if the network is waiting for a user to click, set the selected node, otherwise ignore
                if (!networks.propagating && networks.state == "response") {
                    // set game variables to account for the node being selected and the user starting the propagation
                    networks.propagating = true;
                    networks.nodes[i].selected = true;
                    networks.selectedNode = networks.nodes[i].index;
                    networks.frontierNodes = [i];
                    networks.timeset = Date.now() - networks.propagationPeriod;
                    // log which node was clicked
                    log({"stage": "networks", "puzzle": networks.puzzleName, "action": "node selected", "object": networks.nodes[i].index});
                }
                // stop checking nodes, since we know which one the user clicked
                break;
            }
        }
    }
}
