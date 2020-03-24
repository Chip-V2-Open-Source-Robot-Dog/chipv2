package edu.mit.chip.robotmovement;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.utils.FootPosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryGenerator {
    Leg frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;
    FootPosition FL, FR, BL, BR;

    public TrajectoryGenerator(Leg FLL, Leg FRL, Leg BLL, Leg BRL) {
        if (FLL == null || FRL == null || BLL == null || BRL ==null) {
            throw new IllegalArgumentException("WARNING: One or more leg is missing."); 
        }

        frontLeftLeg = FLL;
        frontRightLeg = FRL;
        backLeftLeg = BLL;
        backRightLeg = BRL;
    }

    public void clearTrajectory() {
        frontLeftLeg.clearTrajectory();
        frontRightLeg.clearTrajectory();
        backLeftLeg.clearTrajectory();
        backRightLeg.clearTrajectory();
    }

    public void trasnferWeight(int direction) {
        if (direction == 1) {
            frontLeftLeg.addPoint(0.05, 0.5, 0.0);
            frontRightLeg.addPoint(0.05, 0.5, 0.0);
            backLeftLeg.addPoint(0.1, 0.45, 0.0);
            backRightLeg.addPoint(0.1, 0.45, 0.0);
        }
        if (direction == -1) {
            frontLeftLeg.addPoint(-0.1, 0.45, 0.0);
            frontRightLeg.addPoint(-0.1, 0.45, 0.0);
            backLeftLeg.addPoint(-0.05, 0.5, 0.0);
            backRightLeg.addPoint(-0.05, 0.5, 0.0);
        }

        boolean done = false;
        while (!done) {
            done = move(0.05);
        }
    }

    public void genStep(String leg, double speed) {
        FL = frontLeftLeg.getFootPosition();
        FR = frontRightLeg.getFootPosition();
        BL = backLeftLeg.getFootPosition();
        BR = backRightLeg.getFootPosition();

        if (leg.equals("FL")) {
            frontLeftLeg.addPoint(-0.1, 0.35, 0.0);
            frontLeftLeg.addPoint(-0.2, 0.35, 0.0);
            frontLeftLeg.addPoint(-0.2, 0.45, 0.0);
        }

        if (leg.equals("FR")) {
            frontRightLeg.addPoint(-0.1, 0.35, 0.0);
            frontRightLeg.addPoint(-0.2, 0.35, 0.0);
            frontRightLeg.addPoint(-0.2, 0.45, 0.0);
        }

        if (leg.equals("BR")) {
            backRightLeg.addPoint(0.1, 0.35, 0.0);
            backRightLeg.addPoint(0.0, 0.35, 0.0);
            backRightLeg.addPoint(0.0, 0.45, 0.0);
        }

        if (leg.equals("BL")) {
            backLeftLeg.addPoint(0.1, 0.35, 0.0);
            backLeftLeg.addPoint(0.0, 0.35, 0.0);
            backLeftLeg.addPoint(0.0, 0.45, 0.0);
        }

        boolean done = false;
        while (!done) {
            done = move(speed);
        }
    }

    public void lean(String direction) {
        FL = frontLeftLeg.getFootPosition();
        FR = frontRightLeg.getFootPosition();
        BL = backLeftLeg.getFootPosition();
        BR = backRightLeg.getFootPosition();

        if (direction.equals("L")) {
            frontLeftLeg.addPoint(FL.x, FL.y, -0.05);
            frontRightLeg.addPoint(FR.x, FR.y, 0.05);
            backLeftLeg.addPoint(BL.x, BL.y, -0.05);
            backRightLeg.addPoint(BR.x, BR.y, 0.05);
        }
        if (direction.equals("C")) {
            frontLeftLeg.addPoint(FL.x, FL.y, 0.0);
            frontRightLeg.addPoint(FR.x, FR.y, 0.0);
            backLeftLeg.addPoint(BL.x, BL.y, 0.0);
            backRightLeg.addPoint(BR.x, BR.y, 0.0);
        }
        if (direction.equals("R")) {
            frontLeftLeg.addPoint(FL.x, FL.y, 0.05);
            frontRightLeg.addPoint(FR.x, FR.y, -0.05);
            backLeftLeg.addPoint(BL.x, BL.y, 0.05);
            backRightLeg.addPoint(BR.x, BR.y, -0.05);
        }

        boolean done = false;
        while (!done) {
            done = move(0.05);
        }
    }

    public void step(double speed) {
        //FIRST WE NEED TO TRANSFER THE WEIGHT BACKWARDS
        trasnferWeight(-1); 

        lean("L");
        //TAKE THE STEPS
        genStep("FR", speed);
        //ACTUALLY MOVE THE ROBOT SLOWLY
        lean("R");
        //TAKE THE STEPS
        genStep("FL", speed);
        //ACTUALLY MOVE THE ROBOT SLOWLY

        lean("C");
        //FIRST WE NEED TO TRANSFER THE WEIGHT FORWARDS NOW
        trasnferWeight(1); 

        lean("R");
        genStep("BL", speed);
        //ACTUALLY MOVE THE ROBOT SLOWLY
        lean("L");
        genStep("BR", speed);
        //ACTUALLY MOVE THE ROBOT SLOWLY

        lean("C");
    }

    public void genStandSit(double startingY, double finalY, double offset) {
        frontLeftLeg.addPoint(0.0, startingY, 0.0);
        frontRightLeg.addPoint(0.0, startingY, 0.0);
        backLeftLeg.addPoint(0.0+offset, startingY+offset, 0.0);
        backRightLeg.addPoint(0.0+offset, startingY+offset, 0.0);

        frontLeftLeg.addPoint(0.0, finalY, 0.0);
        frontRightLeg.addPoint(0.0, finalY, 0.0);
        backLeftLeg.addPoint(0.0+offset, finalY+offset, 0.0);
        backRightLeg.addPoint(0.0+offset, finalY+offset, 0.0);
    }

    public boolean move(double speed) {
        boolean t1 = frontLeftLeg.move(speed);
        boolean t2 = frontRightLeg.move(speed);
        boolean t3 = backLeftLeg.move(speed);
        boolean t4 = backRightLeg.move(speed);
        return t1 && t2 && t3 && t4;
    }
}


/*

        if (joy.getRawButton(1)) {
            frontLeftLeg.addPoint(SmartDashboard.getNumber("X", 0.0), SmartDashboard.getNumber("Y", 0.0), SmartDashboard.getNumber("Z", 0.0));
        }
        if (joy.getRawButton(2)) {
            backLeftLeg.addPoint(SmartDashboard.getNumber("X", 0.0), SmartDashboard.getNumber("Y", 0.0), SmartDashboard.getNumber("Z", 0.0));
        }
        if (joy.getRawButton(3)) {
            frontRightLeg.addPoint(SmartDashboard.getNumber("X", 0.0), SmartDashboard.getNumber("Y", 0.0), SmartDashboard.getNumber("Z", 0.0));
        }
        if (joy.getRawButton(4)) {
            backRightLeg.addPoint(SmartDashboard.getNumber("X", 0.0), SmartDashboard.getNumber("Y", 0.0), SmartDashboard.getNumber("Z", 0.0));
        }

        frontLeftLeg.move(0.5);
        frontRightLeg.move(0.5);
        backLeftLeg.move(0.5);
        backRightLeg.move(0.5);

        
        frontLeftLeg.home(0.5);
        frontRightLeg.home(0.5);
        backLeftLeg.home(0.5);
        backRightLeg.home(0.5);
        


        frontLeftLeg.clearTrajectory();
        frontRightLeg.clearTrajectory();
        backLeftLeg.clearTrajectory();
        backRightLeg.clearTrajectory();

        SmartDashboard.putNumber("X", 0.0);
        SmartDashboard.putNumber("Y", 0.0);
        SmartDashboard.putNumber("Z", 0.0);
*/