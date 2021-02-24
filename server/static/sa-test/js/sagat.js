class SAGAT {

    constructor(freezePeriod) {
        this.stage = 0;
        this.freezePeriod = freezePeriod * 1000;
        this.freezeTimeset = Date.now();

        this.active = false;

        this.isRunning = false;

        this.selectedGoodWarehouses = [];
        this.selectedBadWarehouses = [];
        this.selectedPendingWarehouses = [];

        this.scores = [];
        this.sagatCount = 1;
        this.sagatMaxFreezes = 6;
    }

    // start the sagat timer
    start() {
        this.freezeTimeset = Date.now();
    }

    // countdown to the next freeze period
    timestep() {
        var time = Date.now();

        // update the spawner if enough time has passed and conditions are met
        if (time - this.freezeTimeset > this.freezePeriod && this.sagatCount <= this.sagatMaxFreezes && this.isRunning) {
            // reset the time
            this.freezeTimeset = time;
            this.freeze();
        }
    }

    // sets the freeze flags and information
    freeze() {
        if (!this.active) {
            this.active = true;
            clearSelectedWarehouses();
            sidePanelSAGATQuestions();
        }
    }

    // resets the freeze timeset
    unfreeze() {
        if (!gameActive) { return; }

        // get the selected warehouses as an array
        var warehouseSelections = [];
        for (var i in warehouses) {
            warehouseSelections.push(warehouses[i].selected);
        }
        this.active = false;  // set the sagat test to inactive
        this.gradeTest(warehouseSelections);
        if (!this.active) {  // if the gradeTest() allows the test to continue
            if (this.sagatCount <= this.sagatMaxFreezes) {
                sidePanelShowInformation();  // change the displayed side panel information
            }
            else {
                sidePanelEndGame(this.scores);  // show the endgame panel information
            }
            this.freezeTimeset = Date.now();  // reset the freeze timer
        }
    }

    // generates a score from the selected warehouse states
    gradeTest(warehouseSelections) {
        var score = 0;
        console.log("-----")
        for (var i in warehouseSelections) {
            // if colored grey, ignore
            if (warehouseSelections[i] == 0) {
                score += 0;
                continue;
            }
            
            // if colored green and warehouse is doing fine
            if (warehouseSelections[i] == 1 && warehouses[i].percentFull() < .7) {
                score += 1;
                continue;
            }
            
            // if colored red and warehouse is failing
            if (warehouseSelections[i] == 2 && warehouses[i].percentFull() > .9) {
                score += 1;
                continue;
            }

            // if colored orange and warehouse looks likely to fail
            if (warehouseSelections[i] == 3 && warehouses[i].percentFull() > .6 && warehouses[i].percentFull() < 1) {
                score += 1;
                continue
            }

            score -= 1;  // penalize wrong answers
        }

        // for the first cycle, check if the warehouses are all green, otherwise write so in the side panel
        console.log(">>>", this.sagatCount, score, warehouseSelections.length)
        if (this.sagatCount == 1 && score != warehouseSelections.length) {
            console.log("messed up the first test .-,", warehouseSelections.length);
            this.active = true;
            this.sagatCount -= 1;
            sidePanelFailedFirstCycle();
        }

        if (this.sagatCount > 1) {  // don't record the first one
            this.scores.push(score)  // record the score
        }
        
        this.sagatCount += 1;  // increment the count\
    }

}