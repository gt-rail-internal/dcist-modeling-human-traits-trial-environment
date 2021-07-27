// gameboard.js: defined the class for the gameboard

// to add your own puzzles, see puzzles.js

class Gameboard {
    constructor() {
        this.context = context;
        this.askId = false;
    }

    // draws the static game board
    draw() {
        // draw each warehouse and paths from it
        for (var i=0; i<networks.nodes.length; i++) {
            // draw the warehouse
            networks.nodes[i].draw();
        }

        // if drawing the connections, draw them
        if (networks.showConnections) {
            networks.drawConnections();
        }
    }
}


