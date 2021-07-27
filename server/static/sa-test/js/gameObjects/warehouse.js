// warehouse.js: defines the class for the warehouse game object

class Warehouse {
    constructor(context, drawFunction, location, color, capacity = 20, height = 50, prefill) {
        this.context = context;  // the canvas context for drawing
        this.drawFunction = drawFunction;  // the function called for drawing the shape
        this.location = location;  // the location of the warehouse
        this.color = color;  // the color of the warehouse
        this.height = height;  // the height of the warehouse
        this.prefill = prefill;  // boolean whether the warehouse is prefilled

        this.downstreamWarehouses = []; // downstream warehouses

        this.count = 0;  // the number of packages currently being stored

        this.storedPackages = [];  // the package objects being stored
        this.capacity = capacity;  // the maximum warehouse capacity
        this.isFull = false;  // whether the warehouse is currently full

        this.selected = 0;  // whether the warehouse is currently selected in the SAGAT test

        this.transferPeriod = 2 * 1000;  // transfer period of 1 second
        this.transferTimeset = Date.now();

    }

    // used to randomly fill a warehouse if desired
    prefillPackages(packages) {
        let packageCount = (Math.random() * this.capacity / 4);  // get a random number of packages
        for (let i = 0; i < packageCount; i++) {
            // create the package
            var newPackage = new Package(this.context, null, null, this.location, null, null);
            newPackage.generatePackage(false, null, null, null);
            // store the package
            this.storedPackages.push(newPackage);
            this.count += 1;
            packages.push(newPackage);
        }

        return packages;
    }

    // returns the portion full as a decimal
    percentFull() {
        return parseFloat(this.count) / parseFloat(this.capacity);
    }

    // called to update the state of the warehouse
    timestep(time) {
        // relay a package in storage if enough time has passed and conditions are met
        if (time - this.transferTimeset > this.transferPeriod) {
            // reset the time
            this.transferTimeset = time;
            // spawn the package
            if (this.storedPackages.length > 0) {
                var sendingPackage = this.storedPackages[0];
                var packageSent = this.relayPackage(sendingPackage);
                if (packageSent) {
                    this.storedPackages.shift();
                    this.count -= 1;
                }
            }
        }

        // if the count exceeds capacity, freeze the lane
        if (this.count >= this.capacity) {
            this.isFull = true;
        }
        else {
            this.isFull = false;
        }
    }

    // draws the warehouse
    draw(sagatActive) {
        // draw the warehouse
        var warehouseColor = sagatActive ? this.selected == 1 ? "green" : this.selected == 2 ? "red" : this.selected == 3 ? "orange" : "lightgrey" : this.color;
        this.drawFunction(this.context, this.location[0], this.location[1], this.height, warehouseColor, this.color);
        // draw the warehouse capacity bar
        this.drawCapacity(sagatActive);
    }

    // draws the capacity bar to the right of the warehouse
    drawCapacity(sagatActive) {
        var indicatorCapacity = this.count / this.capacity;
        var indicatorColor = sagatActive ? "lightgrey" : indicatorCapacity < .7 ? "green" : indicatorCapacity < .90 ? "orange" : indicatorCapacity < 1 ? "red" : "red";  // when full and 90%+, will turn red
        drawRectangle(this.context, this.location[0] + this.height * .75, this.location[1] + this.height / 2 - this.height / 2, 10, this.height, "lightgrey")
        drawRectangle(this.context, this.location[0] + this.height * .75, this.location[1] + this.height / 2 - indicatorCapacity * this.height / 2, 10, indicatorCapacity * this.height, indicatorColor)
    }

    // receive a package that has arrived
    receivePackage(receivedPackage) {
        // if the package is not the correct shape and color, count it and store
        if (receivedPackage.drawFunction != this.drawFunction || receivedPackage.color != this.color) {
            this.count += 1;
            this.storedPackages.push(receivedPackage);
        }
    }

    // send a package to a warehouse that is applicable
    relayPackage(sendingPackage) {
        // split warehouses into suitable and unsuitable locations
        var matchingWarehouses = [];
        var sameShapeWarehouses = [];
        var sameColorWarehouses = [];
        var unsuitableWarehouses = [];
        for (var i = 0; i < this.downstreamWarehouses.length; i++) {
            // check if full (disregard)
            if (this.downstreamWarehouses[i].isFull) {
                continue;
            }
            // check if matching
            if (this.downstreamWarehouses[i].color == sendingPackage.color && this.downstreamWarehouses[i].drawFunction == sendingPackage.drawFunction) {
                matchingWarehouses.push(this.downstreamWarehouses[i]);
            }
            // check if same shape
            else if (this.downstreamWarehouses[i].drawFunction == sendingPackage.drawFunction) {
                sameShapeWarehouses.push(this.downstreamWarehouses[i]);
            }
            // check if same color
            else if (this.downstreamWarehouses[i].color == sendingPackage.color) {
                sameColorWarehouses.push(this.downstreamWarehouses[i]);
            }
            // check if unsuitable
            else {
                unsuitableWarehouses.push(this.downstreamWarehouses[i]);
            }
        }

        // if sending at random, choose a random warehouse from both the suitable and unsuitable warehouses
        if (this.random) {
            sendingPackage.targetWarehouse = randomFromArray(matchingWarehouses.concat(sameShapeWarehouses).concat(sameColorWarehouses).concat(unsuitableWarehouses));
        }
        // if there are suitable warehouses, send to a random one of those
        else if (matchingWarehouses.length > 0) {
            sendingPackage.targetWarehouse = randomFromArray(matchingWarehouses);
        }
        // if there are matching shape warehouses, send to a random one of those
        else if (sameShapeWarehouses.length > 0) {
            sendingPackage.targetWarehouse = randomFromArray(sameShapeWarehouses);
        }
        // if there are matching color warehouses, send to a random one of those
        else if (sameColorWarehouses.length > 0) {
            sendingPackage.targetWarehouse = randomFromArray(sameColorWarehouses);
        }
        // otherwise, send to a random warehouse
        else if (unsuitableWarehouses.length > 0) {
            sendingPackage.targetWarehouse = randomFromArray(unsuitableWarehouses);
        }
        // if none are possible, return false
        else {
            return false;
        }
        // we are sending the package, so make it visible
        sendingPackage.hide = false;
        sendingPackage.atGoal = false;

        // return true by default, indicating success
        return true;
    }
}

// update all the warehouses
function updateWarehouses(warehouses) {
    for (var i = 0; i < warehouses.length; i++) {
        warehouses[i].timestep(Date.now());
    }
}

// deselect all the warehouses
function clearSelectedWarehouses() {
    for (i in gameboard.warehouses) {
        gameboard.warehouses[i].selected = 0;
    }
}