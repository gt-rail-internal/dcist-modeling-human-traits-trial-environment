// puzzles.js contains functions that define the puzzles

// to add a puzzle, copy/paste the generatePuzzle1() function and add/modify the nodes, connections, and game parameters
// then add the function name to the puzzle list in index.html's <script> section

// blank puzzle
function emptyPuzzle() {
    networks.nodes = [];
    networks.index();
}

// generates the puzzle (simple horizontal line)
function generateSimpleLine() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.11, .5];
    var node_2_loc = [.24, .5];
    var node_3_loc = [.37, .5];
    var node_4_loc = [.5, .5];
    var node_5_loc = [.63, .5];
    var node_6_loc = [.76, .5];
    var node_7_loc = [.89, .5];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7];  // set this to include all the nodes

    // fifth set the game parameters
    networks.observationNodes = [0, 2, 5];  // set this to the indexed nodes used in the observation stage
    networks.optimalHops = 4;  // set this to the optimal number of hops (you'll need to figure that out yourself)
    networks.optimalNodes = [3];
    networks.maxRounds = 3;  // set this to the maximum number of guesses the user can make
}

// generates the puzzle (simple arc)
function generateSimpleArc() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.2, .5];
    var node_2_loc = [.2402, .65];
    var node_3_loc = [.35, .7598];
    var node_4_loc = [.5, .8];
    var node_5_loc = [.65, .7598];
    var node_6_loc = [.7598, .65];
    var node_7_loc = [.8, .5];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7];

    // fifth set the game parameters
    networks.observationNodes = [0, 1, 2];
    networks.optimalHops = 4;
    networks.optimalNodes = [3];
    networks.maxRounds = 3;
}

// generates the puzzle (complex sideways arc)
function generateComplexArc() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.1, .77];
    var node_2_loc = [.11, .9];
    var node_3_loc = [.3, .9];
    var node_4_loc = [.32, .80];
    var node_5_loc = [.52, .85];
    var node_6_loc = [.65, .81];
    var node_7_loc = [.8, .89];
    var node_8_loc = [.91, .86];
    var node_9_loc = [.7, .70];
    var node_10_loc = [.88, .68];
    var node_11_loc = [.85, .5];
    var node_12_loc = [.82, .4];
    var node_13_loc = [.87, .30];
    var node_14_loc = [.80, .18];
    var node_15_loc = [.68, .25];
    var node_16_loc = [.6, .14];
    var node_17_loc = [.51, .21];
    var node_18_loc = [.4, .39];
    var node_19_loc = [.42, .27];
    var node_20_loc = [.41, .12];
    var node_21_loc = [.3, .2];
    var node_22_loc = [.21, .12];
    var node_23_loc = [.17, .22];
    var node_24_loc = [.16, .33];
    var node_25_loc = [.16, .42];
    var node_26_loc = [.12, .55];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    var node_9 = new Node(gameboard.context, node_9_loc);
    var node_10 = new Node(gameboard.context, node_10_loc);
    var node_11 = new Node(gameboard.context, node_11_loc);
    var node_12 = new Node(gameboard.context, node_12_loc);
    var node_13 = new Node(gameboard.context, node_13_loc);
    var node_14 = new Node(gameboard.context, node_14_loc);
    var node_15 = new Node(gameboard.context, node_15_loc);
    var node_16 = new Node(gameboard.context, node_16_loc);
    var node_17 = new Node(gameboard.context, node_17_loc);
    var node_18 = new Node(gameboard.context, node_18_loc);
    var node_19 = new Node(gameboard.context, node_19_loc);
    var node_20 = new Node(gameboard.context, node_20_loc);
    var node_21 = new Node(gameboard.context, node_21_loc);
    var node_22 = new Node(gameboard.context, node_22_loc);
    var node_23 = new Node(gameboard.context, node_23_loc);
    var node_24 = new Node(gameboard.context, node_24_loc);
    var node_25 = new Node(gameboard.context, node_25_loc);
    var node_26 = new Node(gameboard.context, node_26_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7);
    node_6.connect(node_9);
    node_7.connect(node_8);
    node_9.connect(node_10);
    node_8.connect(node_10);
    node_10.connect(node_11);
    node_11.connect(node_12);
    node_11.connect(node_13);
    node_12.connect(node_13);
    node_13.connect(node_14);
    node_14.connect(node_15);
    node_14.connect(node_16);
    node_15.connect(node_16);
    node_15.connect(node_17);
    node_16.connect(node_17);
    node_16.connect(node_20);
    node_17.connect(node_18);
    node_17.connect(node_19);
    node_17.connect(node_20);
    node_18.connect(node_21);
    node_19.connect(node_21);
    node_20.connect(node_21);
    node_21.connect(node_22);
    node_21.connect(node_23);
    node_21.connect(node_24);
    node_22.connect(node_23);
    node_23.connect(node_24);
    node_24.connect(node_25);
    node_25.connect(node_26);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9, node_10, node_11, node_12, node_13, node_14, node_15, node_16, node_17, node_18, node_19, node_20, node_21, node_22, node_23, node_24, node_25, node_26];

    // fifth set the game parameters
    networks.observationNodes = [5, 15, 22];
    networks.optimalHops = 9;
    networks.optimalNodes = [15];
    networks.maxRounds = 3;
}


// generates the puzzle (tri star)
function generateTristar() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.07, .92];
    var node_2_loc = [.16, .91];
    var node_3_loc = [.25, .94];
    var node_4_loc = [.34, .92];
    var node_5_loc = [.09, .81];
    var node_6_loc = [.18, .81];
    var node_7_loc = [.27, .81];
    var node_8_loc = [.38, .65];
    var node_9_loc = [.47, .54];
    var node_10_loc = [.52, .43];
    var node_11_loc = [.47, .32];
    var node_12_loc = [.38, .21];
    var node_13_loc = [.27, .18];
    var node_14_loc = [.18, .18];
    var node_15_loc = [.09, .18];
    var node_16_loc = [.09, .10];
    var node_17_loc = [.18, .09];
    var node_18_loc = [.27, .09];
    var node_19_loc = [.36, .11];
    var node_20_loc = [.54, .18];
    var node_21_loc = [.62, .21];
    var node_22_loc = [.70, .24];
    var node_23_loc = [.80, .23];
    var node_24_loc = [.88, .24];
    var node_25_loc = [.96, .21];
    var node_26_loc = [.94, .12];
    var node_27_loc = [.86, .12];
    var node_28_loc = [.78, .14];
    

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    var node_9 = new Node(gameboard.context, node_9_loc);
    var node_10 = new Node(gameboard.context, node_10_loc);
    var node_11 = new Node(gameboard.context, node_11_loc);
    var node_12 = new Node(gameboard.context, node_12_loc);
    var node_13 = new Node(gameboard.context, node_13_loc);
    var node_14 = new Node(gameboard.context, node_14_loc);
    var node_15 = new Node(gameboard.context, node_15_loc);
    var node_16 = new Node(gameboard.context, node_16_loc);
    var node_17 = new Node(gameboard.context, node_17_loc);
    var node_18 = new Node(gameboard.context, node_18_loc);
    var node_19 = new Node(gameboard.context, node_19_loc);
    var node_20 = new Node(gameboard.context, node_20_loc);
    var node_21 = new Node(gameboard.context, node_21_loc);
    var node_22 = new Node(gameboard.context, node_22_loc);
    var node_23 = new Node(gameboard.context, node_23_loc);
    var node_24 = new Node(gameboard.context, node_24_loc);
    var node_25 = new Node(gameboard.context, node_25_loc);
    var node_26 = new Node(gameboard.context, node_26_loc);
    var node_27 = new Node(gameboard.context, node_27_loc);
    var node_28 = new Node(gameboard.context, node_28_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_1.connect(node_5);
    node_2.connect(node_7);
    node_2.connect(node_3);
    node_2.connect(node_4);
    node_2.connect(node_5);
    node_2.connect(node_6);
    node_3.connect(node_7);
    node_3.connect(node_4);
    node_3.connect(node_5);
    node_3.connect(node_6);
    node_4.connect(node_7);
    node_4.connect(node_5);
    node_5.connect(node_7);
    node_6.connect(node_7);
    node_6.connect(node_5);
    node_6.connect(node_2);
    node_6.connect(node_3);
    node_6.connect(node_4);
    node_7.connect(node_8);
    node_8.connect(node_9);
    node_9.connect(node_10);
    node_10.connect(node_11);
    node_11.connect(node_12);
    node_11.connect(node_20);
    node_11.connect(node_21);
    node_12.connect(node_13);
    node_12.connect(node_14);
    node_12.connect(node_18);
    node_12.connect(node_19);
    node_13.connect(node_14);
    node_13.connect(node_18);
    node_13.connect(node_19);
    node_14.connect(node_15);
    node_14.connect(node_16);
    node_14.connect(node_17);
    node_14.connect(node_18);
    node_14.connect(node_19);
    node_15.connect(node_16);
    node_15.connect(node_17);
    node_16.connect(node_17);
    node_20.connect(node_21);
    node_21.connect(node_22);
    node_21.connect(node_23);
    node_21.connect(node_28);
    node_22.connect(node_23);
    node_23.connect(node_24);
    node_23.connect(node_25);
    node_23.connect(node_28);
    node_23.connect(node_27);
    node_24.connect(node_25);
    node_24.connect(node_26);
    node_24.connect(node_27);
    node_25.connect(node_26);
    node_28.connect(node_27);
    node_28.connect(node_26);
    node_28.connect(node_25);
    node_27.connect(node_26);
    node_27.connect(node_25);
    node_27.connect(node_24);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9, node_10, node_11, node_12, node_13, node_14, node_15, node_16, node_17, node_18, node_19, node_20, node_21, node_22, node_23, node_24, node_25, node_26, node_27, node_28];

    // fifth set the game parameters
    networks.observationNodes = [5, 15, 22];
    networks.optimalHops = 6;
    networks.optimalNodes = [8];
    networks.maxRounds = 3;
}

// generates the puzzle (gold standard cross)
function generateSimpleCross() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.2, .5];
    var node_2_loc = [.35, .5];
    var node_3_loc = [.5, .5];
    var node_4_loc = [.65, .5];
    var node_5_loc = [.8, .5];
    var node_6_loc = [.5, .8];
    var node_7_loc = [.5, .65];
    var node_8_loc = [.5, .35];
    var node_9_loc = [.5, .2];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    var node_9 = new Node(gameboard.context, node_9_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_6.connect(node_7);
    node_7.connect(node_3);
    node_3.connect(node_8);
    node_8.connect(node_9);
    node_2.connect(node_7);
    node_2.connect(node_8);
    node_4.connect(node_7);
    node_4.connect(node_8);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9];

    // fifth set the game parameters
    networks.observationNodes = [0, 2, 5];
    networks.optimalHops = 3;
    networks.optimalNodes = [2];
    networks.maxRounds = 3;
}

// generates the puzzle (complicated triangles/squares)
function generateVeryComplexArc() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.1, .77];
    var node_2_loc = [.11, .9];
    var node_3_loc = [.3, .9];
    var node_4_loc = [.32, .80];
    var node_5_loc = [.52, .85];
    var node_6_loc = [.65, .81];
    var node_7_loc = [.8, .89];
    var node_8_loc = [.91, .86];
    var node_9_loc = [.7, .70];
    var node_10_loc = [.88, .68];
    var node_11_loc = [.85, .5];
    var node_12_loc = [.82, .4];
    var node_13_loc = [.87, .30];
    var node_14_loc = [.80, .18];
    var node_15_loc = [.68, .25];
    var node_16_loc = [.6, .14];
    var node_17_loc = [.51, .21];
    var node_18_loc = [.4, .39];
    var node_19_loc = [.42, .27];
    var node_20_loc = [.41, .12];
    var node_21_loc = [.3, .2];
    var node_22_loc = [.21, .12];
    var node_23_loc = [.17, .22];
    var node_24_loc = [.16, .33];
    var node_25_loc = [.16, .42];
    var node_26_loc = [.12, .55];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    node_2.triangle = networks.showTriangles;
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    node_8.triangle = networks.showTriangles;
    var node_9 = new Node(gameboard.context, node_9_loc);
    var node_10 = new Node(gameboard.context, node_10_loc);
    node_10.triangle = networks.showTriangles;
    var node_11 = new Node(gameboard.context, node_11_loc);
    var node_12 = new Node(gameboard.context, node_12_loc);
    var node_13 = new Node(gameboard.context, node_13_loc);
    var node_14 = new Node(gameboard.context, node_14_loc);
    var node_15 = new Node(gameboard.context, node_15_loc);
    node_15.triangle = networks.showTriangles;
    var node_16 = new Node(gameboard.context, node_16_loc);
    var node_17 = new Node(gameboard.context, node_17_loc);
    var node_18 = new Node(gameboard.context, node_18_loc);
    var node_19 = new Node(gameboard.context, node_19_loc);
    node_19.triangle = networks.showTriangles;
    var node_20 = new Node(gameboard.context, node_20_loc);
    var node_21 = new Node(gameboard.context, node_21_loc);
    node_21.triangle = networks.showTriangles;
    var node_22 = new Node(gameboard.context, node_22_loc);
    var node_23 = new Node(gameboard.context, node_23_loc);
    var node_24 = new Node(gameboard.context, node_24_loc);
    var node_25 = new Node(gameboard.context, node_25_loc);
    var node_26 = new Node(gameboard.context, node_26_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7);
    node_6.connect(node_9);
    node_7.connect(node_8);
    node_9.connect(node_10);
    node_8.connect(node_10);
    node_10.connect(node_11);
    node_11.connect(node_12);
    node_11.connect(node_13);
    node_12.connect(node_13);
    node_13.connect(node_14);
    node_14.connect(node_15);
    node_14.connect(node_16);
    node_15.connect(node_16);
    node_15.connect(node_17);
    node_16.connect(node_17);
    node_16.connect(node_20);
    node_17.connect(node_18);
    node_17.connect(node_19);
    node_17.connect(node_20);
    node_18.connect(node_21);
    node_19.connect(node_21);
    node_20.connect(node_21);
    node_21.connect(node_22);
    node_21.connect(node_23);
    node_21.connect(node_24);
    node_22.connect(node_23);
    node_23.connect(node_24);
    node_24.connect(node_25);
    node_25.connect(node_26);

    // the triangle connections
    node_21.connect(node_2);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9, node_10, node_11, node_12, node_13, node_14, node_15, node_16, node_17, node_18, node_19, node_20, node_21, node_22, node_23, node_24, node_25, node_26];

    // fifth set the game parameters
    networks.observationNodes = [5, 10, 22];
    networks.optimalHops = 7;
    networks.optimalNodes = [15];
    networks.maxRounds = 3;
}

// generates the puzzle (large cross)
function generateLinearCross() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.05, .5];
    var node_2_loc = [.2, .5];
    var node_3_loc = [.35, .5];
    var node_4_loc = [.5, .5];
    var node_5_loc = [.65, .5];
    var node_6_loc = [.8, .5];
    var node_7_loc = [.95, .5];
    var node_8_loc = [.5, .95];
    var node_9_loc = [.5, .8];
    var node_10_loc = [.5, .65];
    var node_11_loc = [.5, .35];
    var node_12_loc = [.5, .2];
    var node_13_loc = [.5, .05];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    var node_9 = new Node(gameboard.context, node_9_loc);
    var node_10 = new Node(gameboard.context, node_10_loc);
    var node_11 = new Node(gameboard.context, node_11_loc);
    var node_12 = new Node(gameboard.context, node_12_loc);
    var node_13 = new Node(gameboard.context, node_13_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7);
    node_7.connect(node_8);
    node_8.connect(node_9);
    node_9.connect(node_10);
    node_10.connect(node_11);
    node_11.connect(node_12);
    node_12.connect(node_13);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9, node_10, node_11, node_12, node_13];

    // fifth set the game parameters
    networks.observationNodes = [2, 4, 8];
    networks.optimalHops = 7;
    networks.optimalNodes = [6];
    networks.maxRounds = 3;
}

// generates the puzzle (wavy cross)
function generateWavyLinearCross() {
    // first record the location of each node, as an x% y% of the canvas
    var node_1_loc = [.07, .5];
    var node_2_loc = [.21, .68];
    var node_3_loc = [.35, .78];
    var node_4_loc = [.5, .5];
    var node_5_loc = [.64, .36];
    var node_6_loc = [.78, .5];
    var node_7_loc = [.93, .14];
    var node_8_loc = [.57, .93];
    var node_9_loc = [.60, .78];
    var node_10_loc = [.5, .68];
    var node_11_loc = [.36, .36];
    var node_12_loc = [.74, .21];
    var node_13_loc = [.44, .07];

    // second create the node objects
    var node_1 = new Node(gameboard.context, node_1_loc);
    var node_2 = new Node(gameboard.context, node_2_loc);
    var node_3 = new Node(gameboard.context, node_3_loc);
    var node_4 = new Node(gameboard.context, node_4_loc);
    var node_5 = new Node(gameboard.context, node_5_loc);
    var node_6 = new Node(gameboard.context, node_6_loc);
    var node_7 = new Node(gameboard.context, node_7_loc);
    var node_8 = new Node(gameboard.context, node_8_loc);
    var node_9 = new Node(gameboard.context, node_9_loc);
    var node_10 = new Node(gameboard.context, node_10_loc);
    var node_11 = new Node(gameboard.context, node_11_loc);
    var node_12 = new Node(gameboard.context, node_12_loc);
    var node_13 = new Node(gameboard.context, node_13_loc);

    // third connect the nodes, running a connection from A->B will also create the connection from B->A
    node_1.connect(node_2);
    node_2.connect(node_3);
    node_3.connect(node_4);
    node_4.connect(node_5);
    node_5.connect(node_6);
    node_6.connect(node_7)
    node_7.connect(node_8);
    node_8.connect(node_9);
    node_9.connect(node_10);
    node_10.connect(node_11);
    node_11.connect(node_12);
    node_12.connect(node_13);

    // fourth add the nodes to the network
    networks.nodes = [node_1, node_2, node_3, node_4, node_5, node_6, node_7, node_8, node_9, node_10, node_11, node_12, node_13];

    // fifth set the game parameters
    networks.observationNodes = [2, 4, 8];
    networks.optimalHops = 7;
    networks.optimalNodes = [6];
    networks.maxRounds = 3;
}
