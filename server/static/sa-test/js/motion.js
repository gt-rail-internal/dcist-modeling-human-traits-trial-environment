function moveToPoint(initial_location, target_location, delta) {
    // get the change in x, y, and the normalization factor (hypotenous)
    dx = target_location[0] - initial_location[0]
    dy = target_location[1] - initial_location[1]
    dl = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))
    
    // turn that into the frame step
    motion_dx = delta * dx / dl;
    motion_dy = delta * dy / dl;

    // add the frame step to the current position
    new_location = [initial_location[0] + motion_dx, initial_location[1] + motion_dy]

    return new_location
}