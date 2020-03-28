package edu.mit.chip.trajectory;

import edu.mit.chip.leg.FootPosition;

public class Waypoint {
    FootPosition frontLeft;
    FootPosition frontRight;
    FootPosition backLeft;
    FootPosition backRight;

    public Waypoint(FootPosition frontLeft, FootPosition frontRight, FootPosition backLeft, FootPosition backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }
}