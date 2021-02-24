function degToRad(degrees) {
  return degrees * Math.PI / 180;
};

// draws the path between two points
function drawPath(ctx, x1, y1, x2, y2, width, color) {
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.strokeStyle = color;
  ctx.strokeWidth = width;
  ctx.stroke();
  ctx.strokeStyle = "black";
  ctx.fillStyle = "black";
}

// draws an equilateral triangle
function drawTriangle(ctx, x, y, h, fill, outline) {
  ctx.fillStyle = fill;

  var triangle = new Path2D();

  var l = h * 2 / Math.sqrt(3)
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

// draws a square
function drawSquare(ctx, x, y, h, fill, outline) {
  ctx.fillStyle = fill;
  r = h/2

  var square = new Path2D();

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

// draws a rectangle
function drawRectangle(ctx, x, y, w, l, fill, outline) {
  ctx.fillStyle = fill;
  var rectangle = new Path2D();
  rectangle.moveTo(x - w/2, y - l/2);
  rectangle.lineTo(x + w/2, y - l/2);
  rectangle.lineTo(x + w/2, y + l/2);
  rectangle.lineTo(x - w/2, y + l/2);
  rectangle.closePath();
  ctx.fill(rectangle)
  ctx.fillStyle = "black";

  if (outline) {
    ctx.stroke(rectangle);
  }
}

// draws a pentagon
function drawPentagon(ctx, x, y, h, fill, outline) {
  int_ang = 1.884955592
  r_to_height = 1.80912
  r = h / r_to_height
  l = r * Math.cos(int_ang / 2) * 2
  ctx.fillStyle = fill;
  var pentagon = new Path2D();
  pentagon.moveTo(x, y - r);
  pentagon.lineTo(x + l*Math.cos(.628), y - r + l*Math.sin(.628));
  pentagon.lineTo(x + l/2, y + (r_to_height-1)*r);
  pentagon.lineTo(x - l/2, y + (r_to_height-1)*r);
  pentagon.lineTo(x - l*Math.cos(.628), y - r + l*Math.sin(.628))
  pentagon.closePath();
  ctx.fill(pentagon);
  ctx.fillStyle = "black";

  if (outline) {
    ctx.stroke(pentagon);
  }
}

// draws a circle
function drawCircle(ctx, x, y, h, fill, outline) {
  ctx.fillStyle = fill;
  var circle = new Path2D();
  circle.moveTo(x+h/2, y);
  circle.arc(x, y, h/2, 0, 2 * Math.PI);
  ctx.fill(circle);
  ctx.fillStyle = "black";

  if (outline) {
    ctx.stroke(circle);
  }
}
