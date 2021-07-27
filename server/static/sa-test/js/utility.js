// utility.js: contains utility functions that are used elsewhere

// euclidean distance function
function distance(a, b) {
    return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
}

// gets a random item from an array
function randomFromArray(arr) {
    return arr[Math.floor(rand() * arr.length)]
}

// handles mouse clicks to cycle the warehouse states
function mouseHandler(event) {
    if (event.originalTarget == document.getElementById("Canvas") || event.target == document.getElementById("Canvas")) {
        var canvasX = event.clientX - (event.originalTarget ? event.originalTarget.offsetLeft : event.target.offsetLeft) + window.scrollX;
        var canvasY = event.clientY - (event.originalTarget ? event.originalTarget.offsetTop : event.target.offsetTop) + window.scrollY;
        var canvasClick = [canvasX, canvasY];

        for (i in gameboard.warehouses) {
            if (distance(canvasClick, gameboard.warehouses[i].location) < gameboard.warehouses[i].height * .75) {
                gameboard.warehouses[i].selected = gameboard.warehouses[i].selected == 3 ? 1 : gameboard.warehouses[i].selected + 1;
                selectedObject = i;
                log({"stage": "SAGAT", "action": "canvas-click", "object": i});
                break;
            }
        }
    }
}

// implementation of MurmerHash3 mixing function, taken from (https://stackoverflow.com/a/47593316/3427093)
function xmur3(str) {
    for(var i = 0, h = 1779033703 ^ str.length; i < str.length; i++)
        h = Math.imul(h ^ str.charCodeAt(i), 3432918353),
        h = h << 13 | h >>> 19;
    return function() {
        h = Math.imul(h ^ h >>> 16, 2246822507);
        h = Math.imul(h ^ h >>> 13, 3266489909);
        return (h ^= h >>> 16) >>> 0;
    }
}

// implementation of Mulberry 32-bit pseudorandom number generator, taken from (https://stackoverflow.com/a/47593316/3427093)
function mulberry32(a) {
    return function() {
      var t = a += 0x6D2B79F5;
      t = Math.imul(t ^ t >>> 15, t | 1);
      t ^= t + Math.imul(t ^ t >>> 7, t | 61);
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    }
}

// logs data to the webserver (SET THIS for your own needs)
function log(data) {
    data["worker-id"] = workerId;  // include the worker ID
    return fetch("/logging", {method: "POST", body: JSON.stringify(data)});
}
