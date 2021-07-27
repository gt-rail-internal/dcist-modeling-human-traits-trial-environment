// package.js: contains the package class descriptor and helper functions

// Package core class: a package is a smallish shape that moves along the paths to the warehouses
class Package {
    // base constructor, sets the package attributes
    constructor(context, drawFunction=drawSquare, color="brown", location=[100,100], targetWarehouse=null, speed=2) {
        this.context = context;  // stage context for drawing

        this.height = 20;  // height of the package (integer)
        this.speed = speed;  // speed of the package (integer)
        this.color = color;  // color of the package (HTML codes or string colors)
        this.drawFunction = drawFunction;  // drawing function for the shape (see shapes.js)

        this.location = location;  // current location of the package ([x, y]), note that [0,0] is at the top left
        this.targetWarehouse = targetWarehouse;  // the target location of the package        

        this.hide = false;
        this.atGoal = false;
    }

    // incrementally moves the package to the target destination
    move(targetLocation=null, disappearAtLocation=false) {
        // check if supposed to draw
        if (this.hide || this.atGoal) {
            return;
        }

        // check if package has a target
        if (this.targetWarehouse === null) {
            return;
        }
        else {
            targetLocation = this.targetWarehouse.location;
        }

        // get whether the package is close to the target (within one step)
        var closeToTarget = distance(targetLocation, this.location) < this.speed;

        // if the package is close to the target, set that flag
        if (closeToTarget) {
            this.atGoal = true;
            this.targetWarehouse.receivePackage(this);
            return;
        }

        // check if the package is supposed to disappear at goal
        if (disappearAtLocation & closeToTarget) {
            return;
        }

        // update the package location if the package isn't close to the target and the target's lane isn't frozen
        if (!closeToTarget && !this.targetWarehouse.isFull) {
            this.location = moveToPoint(this.location, targetLocation, this.speed)
        }

        // draw the package
        this.drawFunction(this.context, this.location[0], this.location[1], this.height, this.color, true);

        return;
    }

    // generates package attributes randomly
    generatePackage(colorByShape=false, generateColor=null, generateShape=null, generateSpeed=null) {
        // choose a random color
        if (generateColor === null) {
            var colors = ["red", "blue", "purple", "gold"]
            this.color = colors[Math.floor(rand() * colors.length)];
        }
        else {
            this.color = generateColor;
        }

        // choose a random shape draw function
        if (generateShape === null) {
            var shapes = [drawTriangle, drawSquare, drawPentagon, drawCircle];
            var uniformColors = ["blue", "red", "gold"];  // when coloring uniformly, use these colors
            var shapeIndex = Math.floor(rand() * shapes.length);
            this.drawFunction = shapes[shapeIndex];
            if (colorByShape) {
                this.color = uniformColors[shapeIndex];
            }
        }
        else {
            this.drawFunction = generateShape;
        }

        // choose a random speed from 1-3
        if (generateSpeed === null) {
            this.speed = Math.floor(rand() * 3) + 1;
        }
        else {
            this.speed = generateSpeed;
        }

        return;
    }
}


// move all packages to their built in target locations
function movePackages(packages) {
    packages.forEach((package) => {
        package.move();
    });
    return;
}


// get the indexes of packages that are at their goals
function checkPackagesAtGoal(packages) {
    var states = [];
    // for each package, add its index to the states if it's not at the goal
    for (var i=0; i<packages.length; i++) {
        if (packages[i].atGoal) {
            states.push(i)
        }
    }
    return states;
}


// remove packages that are at the goal
function removePackagesAtGoal(packages) {
    var newPackages = [];  // make a new list for the packages not at the goal
    // for each package, add it to the new list if it's not at the goal
    for (var i=0; i<packages.length; i++) {
        if (!packages[i].atGoal) {
            newPackages.push(packages[i])
        }
    }
    return newPackages;
}


// only get packages that are at the goal
function onlyPackagesAtGoal(packages) {
    var newPackages = [];  // make a new list for the packages at the goal
    // for each package, add it to the new list if it's at the goal
    for (var i=0; i<packages.length; i++) {
        if (packages[i].atGoal) {
            newPackages.push(packages[i])
        }
    }
    return newPackages;
}

