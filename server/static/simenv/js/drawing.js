
// Canvas arrow object
// adapted from https://stackoverflow.com/questions/808826
function canvas_arrow(context, fromx, fromy, tox, toy) {
    var headlen = 10; // length of head in pixels
    var dx = tox - fromx;
    var dy = toy - fromy;
    var angle = Math.atan2(dy, dx);
    context.moveTo(fromx, fromy);
    context.lineTo(tox, toy);
    context.lineTo(tox - headlen * Math.cos(angle - Math.PI / 6), toy - headlen * Math.sin(angle - Math.PI / 6));
    context.moveTo(tox, toy);
    context.lineTo(tox - headlen * Math.cos(angle + Math.PI / 6), toy - headlen * Math.sin(angle + Math.PI / 6));
}

// Adapted from StackOverflow (couldn't find link again :-( )
function drawDashedCircle(context, x, y, r, color) {
    context.strokeStyle = color;
    context.beginPath();
    context.setLineDash([5, 5]);  // set the line stroke to dashed
    context.beginPath();
    context.arc(x, y, r, 0, Math.PI * 2);
    context.closePath();
    context.stroke();
    context.setLineDash([]);  // set the line stroke back to normal
    context.strokeStyle = "black";
}

function drawCircle(context, x, y, r, color) {
    context.strokeStyle = color;
    context.lineWidth = 3;
    context.beginPath();
    context.arc(x, y, r, 0, 2 * Math.PI);
    context.stroke();
    context.strokeStyle = "black";
    context.lineWidth = 2;
}

// draws a rotated image on the canvas
// adapted from https://stackoverflow.com/questions/17411991
function drawRotated(context, image, x, y, scale, rads){
    context.save();
    context.translate(x, y);
    context.rotate(rads);
    context.drawImage(image, -scale/2, -scale/2, scale, scale);
    context.restore();
    return;
}