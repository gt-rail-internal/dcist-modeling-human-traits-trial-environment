// spawner.js: defined the class for the package spawner gameobject

class Spawner {
    constructor(context, location, period, downstreamWarehouses, active=true, random=false, spawnMax=Infinity, colorByShape=false, packageShape=null, packageColor=null, packageSpeed=null) {
        this.context = context;  // drawing context for the shapes
        this.location = location;  // location of the spawner

        this.active = active;  // whether the spawner is spawning
        this.random = random;  // whether the spawner fire randomly or by best warehouses
        this.spawnCount = 0;  // current number of spawns
        this.spawnMax = spawnMax;  // maximum number of spawns
        this.colorByShape = colorByShape;
        this.packageShape = packageShape;  // constant shape for the packages, will be random if null
        this.packageColor = packageColor;  // constant color for the packages, will be random if null
        this.packageSpeed = packageSpeed;  // constant speed for the packages, will be random if null

        this.downstreamWarehouses = downstreamWarehouses;  // potential target locations

        this.spawnerPeriod = period * 1000;  // the time input is in milliseconds, period input in seconds
        this.spawnerTimeSet = Date.now();
    }

    // gets calls on each frame update for time based actions (spawning shapes)
    timestep(time, packages) {
        // update the spawner if enough time has passed and conditions are met
        if (this.active && time - this.spawnerTimeSet > this.spawnerPeriod && this.spawnCount < this.spawnMax) {
            // reset the time
            this.spawnerTimeSet = time;
            // spawn the package
            this.spawn(packages);
        }

        // return the updated packages
        return packages;
    }

    // draws the spawner and paths
    draw() {
        // the spawner is invisible (no icon or anything)
    }

    // spawns one instance of the package
    spawn(packages) {
        if (this.active && this.spawnCount < this.spawnMax) {
            // create a new package
            var newPackage = new Package(this.context, null, null, this.location, null, null);  // create the package
            newPackage.generatePackage(this.colorByShape, this.packageColor, this.packageShape, this.packageSpeed);  // set the package attributes, or random
            
            // send it to a warehouse
            this.sendPackage(newPackage);

            // add it to the packages list
            packages.push(newPackage);

            // update the spawn count
            this.spawnCount += 1;
        }
        return packages;
    }

    // send a package to a warehouse that is applicable
    sendPackage(sendingPackage) {
        // split warehouses into suitable and unsuitable
        var matchingWarehouses = [];
        var sameShapeWarehouses = [];
        var sameColorWarehouses = [];
        var unsuitableWarehouses = [];
        for (var i=0; i<this.downstreamWarehouses.length; i++) {
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

// timestep the spawners
function updateSpawners(spawners, packages) {
    for (var i in spawners) {
        spawners[i].timestep(Date.now(), packages);  // timestep the spawner
    }
}