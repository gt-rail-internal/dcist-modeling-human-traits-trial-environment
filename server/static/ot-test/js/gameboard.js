// gameboard.js: defined the class for the gameboard

// to add your own puzzles, see puzzles.js

class Gameboard {
    constructor(stage, context) {
        this.stage = stage;
        this.context = context;
        this.tracking = 0;  // the number of balls currently being tracked
        this.state = "preparation";
        this.balls = [];
        this.level = -1;  // level will increment each turn

        this.scores = [];

        this.debugLines = false;

        this.freezeTimer = 20;
        this.numSelected = 0;

        this.collisions = [];
    }

    // scores the round by the number of accurately selected balls
    scoreRound() {
        let score = 0;
        for (let i=0; i<this.balls.length; i++) {
            if (this.balls[i].selected && this.balls[i].tracked) {
                score += 1;
            }
        }
        log({"stage": "object tracking", "action": "scoring", "numBalls": this.numBalls, "tracked": this.tracking, "score": score});
        this.scores.push(score);
    }

    // transitions to the next stage
    nextGame() {
        this.level += 1;  // increment the level
        // if still have levels left, run the level
        if (this.level < targetSteps.length) {
            this.tracking = targetSteps[this.level];  // increase the number of balls being tracked
            this.numSelected = 0;
            this.generateGameboard();  // generate the gameboard    
        }
        // otherwise clear the gameboard and show the endgame side panel
        else {
            this.balls = [];
            sidePanelEndGame();
        }
    }

    // generates the object tracking gameboard
    generateGameboard() {
        let width = this.stage.width;
        let height = this.stage.height;

        // clear the gameboard
        this.balls = [];

        // generate the balls
        for (let i = 0; i < numBalls; i++) {
            let angle = i * 2 * Math.PI / numBalls
            this.balls.push(new Ball(width/2 + .3 * width * Math.cos(angle), height/2 + .3 * height * Math.sin(angle)));
            this.balls[i].index = i;
            this.balls[i].speedSetting = ballSpeed;  // set the speed constant
            this.balls[i].initSpeed();  // calculates the resulting speed
            /*let forcedAngle = 0;  // for when you want to force angles between the balls
            if (i == 0) {
                this.balls[i].angle = forcedAngle;
            }
            if (i == 1) {
                this.balls[i].angle = Math.PI - forcedAngle;
            }*/
        }

        let usedIndexes = [];

        // select random balls to track
        for (let i = 0; i < this.tracking; i++) {
            // get the index
            let index = Math.floor(rand() * numBalls);
            while (usedIndexes.includes(index)) {
                index = Math.floor(rand() * numBalls);
            }
            usedIndexes.push(index);

            // set the color
            this.balls[index].color = "blue";

            // set the tracked state
            this.balls[index].tracked = true;
        }
    }

    // starts moving the balls
    start() {
        this.timestepStart = Date.now() / 1000;
        this.timestepOld = this.timestepStart;
        this.freezeStart = Date.now();
    }

    // moves the balls
    move() {
        // update the timestep values
        let now = Date.now() / 1000;
        let timestep = now - this.timestepOld;
        this.timestepOld = now;

        // reset collisions
        this.collisions = [];

        // move each ball
        for (var i = 0; i < this.balls.length; i++) {
            this.balls[i].move(timestep);
        }

        // if the freeze timer is done, stop moving and show the response panel
        if (Date.now() - this.freezeStart >= this.freezeTimer * 1000) {
            // show the response panel
            this.state = "response";
            sidePanelResponse();
        }
    }

    // draws the static game board
    draw() {
        // draw each ball
        for (var i = 0; i < this.balls.length; i++) {
            // draw the warehouse
            this.balls[i].draw();
        }
    }

}


