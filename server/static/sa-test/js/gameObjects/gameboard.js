// gameboard.js: defines the class for the gameboard

class Gameboard {
    constructor(context, warehouses=[], spawners=[]) {
        this.context = context;
        this.warehouses = warehouses;
        this.spawners = spawners;
    }

    // generates the warehouse objects
    generateGameboard() {
        var layer_1_A_loc = [300, 500];
        var layer_1_B_loc = [600, 500];
        var layer_1_C_loc = [900, 500];

        // create a warehouse for each shape
        var layer_1_A = new Warehouse(this.context, drawTriangle, layer_1_A_loc, "blue", 20, 50, true);
        var layer_1_B = new Warehouse(this.context, drawSquare, layer_1_B_loc, "red", 20, 50, true);
        var layer_1_C = new Warehouse(this.context, drawPentagon, layer_1_C_loc, "gold", 20, 50, true);

        // create a second layer of warehouses
        var layer_2_A_loc = [200, 300];
        var layer_2_B_loc = [300, 300];
        var layer_2_C_loc = [500, 300];
        var layer_2_D_loc = [600, 300];
        var layer_2_E_loc = [800, 300];
        var layer_2_F_loc = [900, 300];
        var layer_2_G_loc = [1000, 300];

        var layer_2_A = new Warehouse(this.context, drawSquare, layer_2_A_loc, "red", 20, 50, true);
        var layer_2_B = new Warehouse(this.context, drawTriangle, layer_2_B_loc, "blue", 20, 50, true);
        var layer_2_C = new Warehouse(this.context, drawCircle, layer_2_C_loc, "gold", 20, 50, true);
        var layer_2_D = new Warehouse(this.context, drawTriangle, layer_2_D_loc, "blue", 20, 50, true);
        var layer_2_E = new Warehouse(this.context, drawSquare, layer_2_E_loc, "red", 20, 50, true);
        var layer_2_F = new Warehouse(this.context, drawPentagon, layer_2_F_loc, "purple", 20, 50, true);
        var layer_2_G = new Warehouse(this.context, drawSquare, layer_2_G_loc, "gold", 20, 50, true);


        // create a third layer of warehouses
        var layer_3_A_loc = [100, 100];
        var layer_3_B_loc = [250, 100];
        var layer_3_C_loc = [450, 100];
        var layer_3_D_loc = [600, 100];
        var layer_3_E_loc = [800, 100];
        var layer_3_F_loc = [950, 100];
        var layer_3_G_loc = [1050, 100];

        var layer_3_A = new Warehouse(this.context, drawPentagon, layer_3_A_loc, "red", 20, 50, true);
        var layer_3_B = new Warehouse(this.context, drawTriangle, layer_3_B_loc, "gold", 20, 50, true);
        var layer_3_C = new Warehouse(this.context, drawSquare, layer_3_C_loc, "purple", 20, 50, true);
        var layer_3_D = new Warehouse(this.context, drawSquare, layer_3_D_loc, "gold", 20, 50, true);
        var layer_3_E = new Warehouse(this.context, drawTriangle, layer_3_E_loc, "purple", 20, 50, true);
        var layer_3_F = new Warehouse(this.context, drawPentagon, layer_3_F_loc, "blue", 20, 50, true);
        var layer_3_G = new Warehouse(this.context, drawCircle, layer_3_G_loc, "red", 20, 50, true);

        layer_1_A.downstreamWarehouses.push(layer_2_A, layer_2_B);
        layer_1_B.downstreamWarehouses.push(layer_2_B, layer_2_C, layer_2_D, layer_2_E);
        layer_1_C.downstreamWarehouses.push(layer_2_E, layer_2_F, layer_2_G);

        layer_2_A.downstreamWarehouses.push(layer_3_A, layer_3_B)
        layer_2_B.downstreamWarehouses.push(layer_3_B)
        layer_2_C.downstreamWarehouses.push(layer_3_C)
        layer_2_D.downstreamWarehouses.push(layer_3_D)
        layer_2_E.downstreamWarehouses.push(layer_3_D, layer_3_E)
        layer_2_F.downstreamWarehouses.push(layer_3_F, layer_3_G)
        layer_2_G.downstreamWarehouses.push(layer_3_G)
        
        // record each warehouse in the gameboard object
        this.warehouses = [layer_1_A, layer_1_B, layer_1_C,
                           layer_2_A, layer_2_B, layer_2_C, layer_2_D, layer_2_E, layer_2_F, layer_2_G,
                           layer_3_A, layer_3_B, layer_3_C, layer_3_D, layer_3_E, layer_3_F, layer_3_G];

        // generate the base package spawner
        // context, location, period, downstreamWarehouses, active=true, random=false, spawnMax=Infinity, colorByShape=false, packageShape=null, packageColor=null, packageSpeed=null
        var baseSpawner = new Spawner(context, [600, 700], .5, [layer_1_A, layer_1_B, layer_1_C], true, false, Infinity, false, undefined, undefined, undefined);
        this.spawners.push(baseSpawner);

        packages = [];  // active packages
        warehouses = this.warehouses;  // active warehouses
        spawners = this.spawners;  // active spawners

        layer_1_A.prefillPackages(packages);
        layer_1_B.prefillPackages(packages);
        layer_1_C.prefillPackages(packages);

        layer_2_A.prefillPackages(packages);
        layer_2_B.prefillPackages(packages);
        layer_2_C.prefillPackages(packages);
        layer_2_D.prefillPackages(packages);
        layer_2_E.prefillPackages(packages);
        layer_2_F.prefillPackages(packages);
        layer_2_G.prefillPackages(packages);

        layer_3_A.prefillPackages(packages);
        layer_3_B.prefillPackages(packages);
        layer_3_C.prefillPackages(packages);
        layer_3_D.prefillPackages(packages);
        layer_3_E.prefillPackages(packages);
        layer_3_F.prefillPackages(packages);
        layer_3_G.prefillPackages(packages);
    }

    // create the tutorial gameboard
    generateTutorialGameboard() {
        var layer_1_loc = [600, 400];
        var layer_1 = new Warehouse(this.context, drawPentagon, layer_1_loc, "purple", 20, 50, false);

        var layer_2_A_loc = [500, 200];
        var layer_2_A = new Warehouse(this.context, drawSquare, layer_2_A_loc, "gold", 20, 50, false);

        var layer_2_B_loc = [700, 200];
        var layer_2_B = new Warehouse(this.context, drawCircle, layer_2_B_loc, "red", 20, 50, false);

        layer_1.downstreamWarehouses.push(layer_2_A, layer_2_B);

        this.warehouses = [layer_1, layer_2_A, layer_2_B];

        var baseSpawner = new Spawner(context, [600, 700], 1, [layer_1], true, false, Infinity, false, undefined, undefined, undefined);
        this.spawners.push(baseSpawner);

        packages = [];  // active packages
        warehouses = this.warehouses;  // active warehouses
        spawners = this.spawners;  // active spawners
    }

    // draws the static game board
    draw(sagatActive=false) {
        // draw each spawner and paths from it
        for (var i=0; i<this.spawners.length; i++) {
            this.spawners[i].draw();

            // draw the paths to child warehouses
            for (var j in this.spawners[i].downstreamWarehouses) {
                var pathColor = this.spawners[i].downstreamWarehouses[j].isFull ? "red" : "black";
                drawPath(this.context, this.spawners[i].location[0], this.spawners[i].location[1], this.spawners[i].downstreamWarehouses[j].location[0], this.spawners[i].downstreamWarehouses[j].location[1], 2, pathColor);
            }
        }

        // draw each warehouse and paths from it
        for (var i=0; i<this.warehouses.length; i++) {          
            // draw the paths to child warehouses
            for (var j in this.warehouses[i].downstreamWarehouses) {
                var pathColor = (this.warehouses[i].downstreamWarehouses[j].isFull && !sagatActive) ? "red" : "black";
                drawPath(this.context, this.warehouses[i].location[0], this.warehouses[i].location[1], this.warehouses[i].downstreamWarehouses[j].location[0], this.warehouses[i].downstreamWarehouses[j].location[1], 2, pathColor);
            }

            // draw the warehouse
            this.warehouses[i].draw(sagatActive);
        }
    }

    // used to get the locations of each warehouse
    getWarehouseLocations() {
        var locations = [];
        for (var i=0; i<this.warehouses.length; i++) {
            locations.push(this.warehouses[i].location);
        }
        return locations;
    }
    
    // returns the warehouses directly connected to the base spawner
    getWarehousesFirstLayer() {
        return 
    }
    
}


