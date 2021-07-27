// Networks.js contains the Networks class, which manages all the network propagation, scoring, etc

class Networks {

    constructor(startingNetwork) {
        this.puzzle = startingNetwork;  // the index of the puzzle (network)
        this.puzzleName = "none";  // the name of the puzzle, for logging
        this.propagationPeriod = 750;  // the wait time (in miliseconds) between propagations, by default this is 750

        this.state = "observation";  // the state: observation (user watches network examples), response (user chooses a node to propagate)
        this.propagating = false;  // indicates whether the network is propagating, users can only select a node if this is false
        this.endRound = false;  // indicates whether this round has ended (nodes turn green)

        this.selections = [];  // the selected nodes for the puzzle
        this.selectedNode = null;  // the index of the selected root node

        this.puzzles = [];  // list of functions to generate puzzles (function[])
        this.nodes = [];  // list of the nodes (Node[])

        this.observationNodes = [];  // list of the node indexes we are observing in the observation state (Int[])
        this.frontierNodes = [];  // list of the node indexes at the frontier of the propagation (Int[])
        this.seenNodes = [];  // list of the node indexes that the propagation has already flipped (Int[])

        this.scores = [];  // list of the scores from this puzzle (Int[]), scores are the number of hops by the user's selection, minus the optimal number of hops
        this.hops = 0;  // the number of hops in this round of propagation
        this.optimalHops = 0;  // the optimal number of hops for this puzzle
        this.optimalNodes = [];  // the optimal nodes, for the debugging features that highlights them
        
        this.maxRounds = 3  // the maximum number of rounds allowed for the user response, by default this is 3
        this.round = 0;  // the current response round of this puzzle, each puzzle has up to three response rounds
        this.showScore = false;  // indicates whether the user score will be shown at the end

        this.showConnections = false;  // flag to display the connections between nodes, used for visually confirming the network structures
        this.showTriangles = true;  // flag to display some networks as triangles, as desired
        this.showOptimalNodes = false;  // flag to display the optimal nodes in purple, for debugging
    }

    // start the propagation
    start() {
        this.timeset = Date.now();
    }

    // sets the .index variable of each node to its index in the this.nodes[] array
    index() {
        for (var i=0; i<this.nodes.length; i++) {
            this.nodes[i].index = i;
        }
    }

    // handles time-based events, such as the periodic network propagation
    timestep() {
        var time = Date.now();

        // propagate the network
        if (time - this.timeset > this.propagationPeriod && (this.state == "observation" || this.state == "response")) {
            // reset the time
            this.timeset = time;
            this.propagate();
        }
    }

    // propagates the network one step
    propagate() {
        // don't propagate if the round is complete
        if (this.endRound) {
            return
        }

        // if all nodes are already propagated, move to the next round
        if (this.propagating && this.frontierNodes.length == 0) {
            this.propagating = false;
            this.nextRound();  // resets the network and checks the completion conditions
            return
        }

        // if in the observation state and not propagating, set the selected node to the next observation node
        if (!this.propagating && this.state == "observation") {
            // sets the frontier to the first element of the observation nodes, then removes that element, so when list is empty we are ready for the user response
            this.frontierNodes = [this.observationNodes[0]];  // by setting a single-element frontier nodes we set a root node for propagation
            this.observationNodes.splice(0, 1);  // remove the first element of the observation node
            this.propagating = true;
        }

        // if propagation is active and the user is observing/responding, flip the frontier nodes and establish the new frontier
        if (this.propagating && (this.state == "observation" || this.state == "response") && this.frontierNodes.length != 0) {
            // flip all the frontier nodes and add them to the seen node list
            for (var i=0; i<this.frontierNodes.length; i++) {
                this.nodes[this.frontierNodes[i]].flip();
                this.seenNodes.push(parseInt(this.frontierNodes[i]));
            }

            // add the nodes connected to the frontier nodes (and not already seen) to a new frontier
            var newFrontier = [];
            for (var i=0; i<this.frontierNodes.length; i++) {
                var newNodes = this.nodes[this.frontierNodes[i]].connections;
                for (var j=0; j<newNodes.length; j++) {
                    if (!(this.seenNodes.includes(newNodes[j].index))) {
                        newFrontier.push(parseInt(newNodes[j].index));
                    }
                }
            }
            this.frontierNodes = newFrontier;

            this.hops += 1;  // record the number of hops we have now done
        }
    }

    // resets the network, checks completion conditions, and changes the observation/response state if needed
    nextRound() {
        // if we are in the user response state, record the score
        if (this.state == "response") {
            this.scores.push(this.hops - this.optimalHops);
            log({"stage": "networks", "action": "score from round #" + this.round, "score": this.hops - this.optimalHops, "name": this.puzzleName});

            this.selections.push(this.selectedNode);
            this.round += 1;
            log({"stage": "networks", "action": "ready for node selection #" + this.round, "name": this.puzzleName});
        }

        // either way, unflip all the nodes
        for (var i=0; i<this.nodes.length; i++) {
            this.nodes[i].unflip();
        }

        // if we are in the user response state and the puzzle is complete (max rounds or optimal hops achieved), end the puzzle
        if (this.state == "response" && (this.round == this.maxRounds || this.hops == this.optimalHops)) {
            // if there are spare rounds, add 0 scores to the scores so it's a consistent maxRounds scores per puzzle
            if (this.round < this.maxRounds) {
                for (var i=this.round; i<this.maxRounds; i++) {
                    this.scores.push(0);
                    this.selections.push(-1);
                }
            }
            this.complete();  // set the nodes to completion
        }

        // reset the propagation variables
        this.seenNodes = [];
        this.frontierNodes = [];
        this.selectedNode = null;
        
        // if we are in the observation state and have no more observations to see, switch to the response state
        if (this.state == "observation" && this.observationNodes.length == 0) {
            this.state = "response";
            log({"stage": "networks", "action": "ready for node selection #" + this.round, "name": this.puzzleName});
            sidePanelResponse();  // display the user response side panel
        }

        // reset the hops counter
        this.hops = 0; 
    }

    // complete the network
    complete() {
        // set all the nodes to "complete" (green)
        this.endRound = true;
        for (var i=0; i<this.nodes.length; i++) {
            this.nodes[i].complete();
        }

        log({"stage": "networks", "action": "puzzle complete", "name": this.puzzleName, "selections": this.selections});  // log the user's puzzle scores

        sidePanelPuzzleComplete();  // display the round complete side panel
    }

    // move on to the next puzzle
    nextPuzzle() {
        if (this.puzzle < this.puzzles.length) {  // if we still have puzzles left, generate the next one
            this.puzzles[this.puzzle]();  // run the function that sets up the next puzzle, established in index.html
            // set up the optimal nodes
            if (this.showOptimalNodes) {
                for (let i = 0; i < this.optimalNodes.length; i++) {
                    this.nodes[this.optimalNodes[i]].optimalNode = true;
                }
            }
            this.puzzleName = this.puzzles[this.puzzle].name;
            this.propagating = true;  // start the next puzzle
            this.index();  // set the .index class variable of each node to their index in the networks.nodes array
            sidePanelObservation();  // display the observation side panel
        }
        else {  // otherwise end the game
            emptyPuzzle();  // set the gameboard to blank
            let score = this.scores.reduce((a, b) => a + b, 0);  // calculate the user's score
            log({"stage": "networks", "action": "game complete", "score": score, "scores": this.scores});  // log the user's total score and scores list
            sidePanelEndGame(score);  // display the end game side panel
        }

        // reset the puzzle parameters
        this.puzzle += 1;
        this.endRound = false;
        this.round = 0;
        this.state = "observation";
        this.timeset = Infinity;
        this.selections = [];
    }

    drawConnections() {
        // for each node
        for (let i=0; i<this.nodes.length; i++) {
            let width = stage.width;
            let height = stage.height;
            // for each connection to that node
            for (let j=0; j<this.nodes[i].connections.length; j++) {
                // draw a line between the nodes
                this.nodes[i].context.beginPath();
                this.nodes[i].context.strokeStyle = "black";
                this.nodes[i].context.moveTo(this.nodes[i].location[0] * width, height - this.nodes[i].location[1] * height);
                this.nodes[i].context.lineTo(this.nodes[i].connections[j].location[0] * width, height - this.nodes[i].connections[j].location[1] * height);
                this.nodes[i].context.stroke();
            }
        }
    }
}