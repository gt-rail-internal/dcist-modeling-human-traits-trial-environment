// visual ball class that the user watches
class Ball {
    constructor(x, y) {
        this.x = x;  // float
        this.y = y;  // float

        this.angle = undefined;
        this.initAngle();

        this.height = 75;

        this.color = "red";

        this.speedSetting = 0.08; // the speed constant
        this.speed = undefined;
        this.initSpeed();

        this.tracked = false;  // tracked
        this.active = true;

        this.index = undefined;

        this.selected = false;

        this.printed = false;
    }

    initAngle() {
        this.angle = rand() * 2 * Math.PI;
    }

    initSpeed() {
        this.speed = .12 * Math.sqrt(Math.pow(stage.width, 2) + Math.pow(stage.height, 2));
        return
    }

    // draws a circle
    draw() {
        gameboard.context.fillStyle = this.selected ? "orange" : this.color;
        var circle = new Path2D();
        gameboard.context.strokeStyle = "black";
        gameboard.context.shadowColor = "black";
        gameboard.context.shadowBlur = 2;
        gameboard.context.shadowOffsetX = 0;
        gameboard.context.shadowOffsetY = 0;
        circle.moveTo(this.x + this.height / 2, this.y);
        circle.arc(this.x, this.y, this.height / 2, 0, 2 * Math.PI);
        gameboard.context.fill(circle);
        gameboard.context.fillStyle = "black";

        gameboard.context.stroke(circle);

        // debug lines
        if (gameboard.debugLines && (this.index == 0 || this.index == 1)) {
            // draw a direction line
            gameboard.context.beginPath();
            gameboard.context.strokeStyle = "black";
            gameboard.context.moveTo(this.x, this.y);
            gameboard.context.lineTo(this.x + 100 * Math.cos(this.angle), this.y + 100 * Math.sin(this.angle));
            gameboard.context.stroke();

            // draw a connection line
            gameboard.context.beginPath();
            gameboard.context.strokeStyle = "blue";
            gameboard.context.moveTo(gameboard.balls[0].x, gameboard.balls[0].y);
            gameboard.context.lineTo(gameboard.balls[1].x, gameboard.balls[1].y);
            gameboard.context.stroke();

            // draw the axis line
            gameboard.context.beginPath();
            gameboard.context.strokeStyle = "green";
            let i = this.index == 0 ? 1 : 0;
            let averageLocation = [this.x / 2 + gameboard.balls[i].x / 2, this.y / 2 + gameboard.balls[i].y / 2];
            let combinedAngle = Math.atan2(this.y - gameboard.balls[i].y, this.x - gameboard.balls[i].x);
            gameboard.context.moveTo(averageLocation[0], averageLocation[1]);
            gameboard.context.lineTo(averageLocation[0] + 100 * Math.cos(combinedAngle - Math.PI / 2), averageLocation[1] + 100 * Math.sin(combinedAngle - Math.PI / 2));
            gameboard.context.stroke();

            // draw 180 axis line
            gameboard.context.beginPath();
            gameboard.context.strokeStyle = "goldenrod";
            gameboard.context.moveTo(this.x, this.y);
            gameboard.context.lineTo(this.x + 100 * Math.cos(combinedAngle), this.y + 100 * Math.sin(combinedAngle));
            gameboard.context.stroke();

            // draw the collision direction line
            // difference between angles
            let angleDiff = combinedAngle - this.angle;
            // determine the bounce
            let newAngle = (combinedAngle + angleDiff + Math.PI) % (2 * Math.PI);
            gameboard.context.beginPath();
            gameboard.context.strokeStyle = "cyan";
            gameboard.context.moveTo(this.x, this.y);
            gameboard.context.lineTo(this.x + 100 * Math.cos(newAngle), this.y + 100 * Math.sin(newAngle));
            gameboard.context.stroke();
            if (this.index == 0) {
                //console.log(this.index, ">>>>>>", newAngle);
            }
        }

    }

    // move the ball
    move(timestep) {
        if (this.printed) {
            //return
        }

        // check collisions to change angles if needed
        for (let i = 0; i < gameboard.balls.length; i++) {
            if (i == this.index) {
                continue;
            }

            // check collision with another ball
            if (distance([this.x, this.y], [gameboard.balls[i].x, gameboard.balls[i].y]) <= this.height) {
                this.calcAngle(gameboard.balls[i].x, gameboard.balls[i].y, gameboard.balls[i].angle);
                gameboard.balls[i].calcAngle(this.x, this.y);
                this.printed = true;

                let combinedAngle = Math.atan2(this.y - gameboard.balls[i].y, this.x - gameboard.balls[i].x);
                this.x = gameboard.balls[i].x + (gameboard.balls[i].height + 2) * Math.cos(combinedAngle) 
                this.y = gameboard.balls[i].y + (gameboard.balls[i].height + 2) * Math.sin(combinedAngle)

            }
        }

        // check if at bounds
        // top wall
        if (distance([this.x, this.y], [this.x, 0]) <= this.height / 2) {
            this.calcAngle(this.x, 0, -Math.PI/2);
            this.y = this.height / 2;
        }

        // bottom wall
        if (distance([this.x, this.y], [this.x, stage.height]) <= this.height / 2) {
            this.calcAngle(this.x, stage.height, Math.PI/2);
            this.y = stage.height - this.height / 2;
        }

        // left wall
        if (distance([this.x, this.y], [0, this.y]) <= this.height / 2) {
            this.calcAngle(0, this.y, 0);
            this.x = this.height / 2;
        }

        // right wall
        if (distance([this.x, this.y], [stage.width, this.y]) <= this.height / 2) {
            this.calcAngle(stage.width, this.y, Math.PI);
            this.x = stage.width - this.height / 2;
        }

        // move the ball
        let dX = this.speed * Math.cos(this.angle) * timestep;
        let dY = this.speed * Math.sin(this.angle) * timestep;

        this.x += dX;
        this.y += dY;

    }

    // calculate the new angle from the collision
    calcAngle(otherX, otherY, otherAngle) {

        // angle between balls
        let combinedAngle = Math.atan2(this.y - otherY, this.x - otherX);

        // perpendicular 
        let axisAngle = 0;
        if (combinedAngle + Math.PI / 2 - this.angle < combinedAngle - Math.PI / 2 - this.angle) {
            axisAngle = combinedAngle + Math.PI / 2;
        }
        else {
            axisAngle = combinedAngle - Math.PI / 2;
        }

        // difference between angles
        let angleDiff = combinedAngle - this.angle;

        // determine the bounce
        let newAngle = (combinedAngle + angleDiff + Math.PI) % (2 * Math.PI);

        // determine whether to add 180 to the angle
        angleDiff = getAngleDiff(newAngle, combinedAngle);
        
        this.angle = newAngle;
    }
}