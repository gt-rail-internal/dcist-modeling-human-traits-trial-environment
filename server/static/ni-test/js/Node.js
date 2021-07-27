// Node.js contains the Node class and its supporting functions
class Node {
    constructor(context, location) {
        this.context = context;
        this.location = location;
        this.height = 50;  // this height is hard-coded, but you can change this if you want
        this.strokeWidth = 7;  // this stroke width is hard-coded, but you can change this if you want
        this.selected = false;

        this.connections = [];

        this.index = null;
        
        this.flipped = false;
        this.green = false;

        this.triangle = false;  // indicates whether the node is a triangle, false by default
        this.optimalNode = false;  // indicates whether the node is an optimal node, in which case color it purple
    }

    // flips the node, will display it blue
    flip() {
        this.green = false;
        this.flipped = true;
    }
  
    // unflip the node, will display it red
    unflip() {
        this.green = false;
        this.flipped = false;
    }

    // complete the node, will display it green
    complete() {
        this.green = true;
    }

    // add a connection between this node and another node, this is a two-directional function
    connect(node) {
        // only add the connection if it doesn't exist
        if (!this.connections.includes(node)) {
            this.connections.push(node);
        }
        if (!node.connections.includes(this)) {
            node.connections.push(this);
        }
    }

    // draws the node on the gameboard
    draw() {
        // determine the node color
        var color = this.green ? "lightgreen" : this.index == networks.selectedNode ? "goldenrod" : this.flipped ? "blue" : this.optimalNode ? "purple" : "red";
        // if the node is a triangle, draw a triangle
        if (this.triangle) {
            this.drawTriangle(this.context, this.location[0] * this.context.canvas.width, (1-this.location[1]) * this.context.canvas.height, this.height, "", color);
        }
        // otherwise draw a square
        else {
            this.drawSquare(this.context, this.location[0] * this.context.canvas.width, (1-this.location[1]) * this.context.canvas.height, this.height, "", color);
        }
    }

    // draws a square
    drawSquare(ctx, x, y, h, fill, outline) {
        ctx.fillStyle = fill != "" ? fill : "rgba(0, 0, 0, 0)";
        ctx.strokeStyle = outline != "" ? outline : "rgba(0, 0, 0, 0)";
        ctx.lineWidth = this.strokeWidth;
        let r = h/2;
    
        let square = new Path2D();
    
        ctx.beginPath();
        square.moveTo(x - r, y - r);
        square.lineTo(x + r, y - r);
        square.lineTo(x + r, y + r);
        square.lineTo(x - r, y + r);
        square.closePath();
        ctx.fill(square)
        ctx.fillStyle = "black";
    
        if (outline) {
            ctx.stroke(square);
        }
    }

    // draws an equilateral triangle
    drawTriangle(ctx, x, y, h, fill, outline) {
        ctx.fillStyle = fill != "" ? fill : "rgba(0, 0, 0, 0)";
        ctx.strokeStyle = outline != "" ? outline : "rgba(0, 0, 0, 0)";
        ctx.lineWidth = this.strokeWidth;
    
        let triangle = new Path2D();
    
        let l = h * 2 / Math.sqrt(3)
        triangle.moveTo(x, y - h*1/2)
        triangle.lineTo(x + l/2, y + h * 1/3 + h/6)
        triangle.lineTo(x - l/2, y + h * 1/3 + h/6)
        triangle.closePath()
        
        ctx.fill(triangle)
        ctx.fillStyle = "black";
    
        if (outline) {
            ctx.stroke(triangle);
        }
    }
}