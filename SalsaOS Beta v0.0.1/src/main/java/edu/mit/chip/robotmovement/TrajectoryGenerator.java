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

    public void genStandSit(double startingY, double finalY, double offset) {
        frontLeftLeg.addPoint(0.0, startingY, 0.0);
        frontRightLeg.addPoint(0.0, startingY, 0.0);
        backLeftLeg.addPoint(0.0, startingY+offset, 0.0);
        backRightLeg.addPoint(0.0, startingY+offset, 0.0);

        frontLeftLeg.addPoint(0.0, finalY, 0.0);
        frontRightLeg.addPoint(0.0, finalY, 0.0);
        backLeftLeg.addPoint(0.0, finalY+offset, 0.0);
        backRightLeg.addPoint(0.0, finalY+offset, 0.0);
    }

    public void lean(double rollAngle, double robotWidth) {
        if (Math.abs(rollAngle) >= Math.PI/12) {
            rollAngle = Math.signum(rollAngle)*Math.PI/12;
        }

        double YL_ADDER = 0.5*robotWidth*Math.sin(rollAngle);
        double YR_ADDER = -1.0*YL_ADDER;

        FL = frontLeftLeg.getFootPosition();
        FR = frontRightLeg.getFootPosition();
        BR = backRightLeg.getFootPosition();
        BL = backLeftLeg.getFootPosition();

        double ZL_MULTIPLIER = -1.0*Math.sin(rollAngle);
        double ZR_MULTIPLIER = Math.sin(rollAngle);

        frontLeftLeg.addPoint(FL.x, FL.y+YL_ADDER, FL.z+(FL.y+YL_ADDER)*ZL_MULTIPLIER);
        frontRightLeg.addPoint(FR.x, FR.y+YR_ADDER, FR.z+(FR.y+YR_ADDER)*ZR_MULTIPLIER);
        backRightLeg.addPoint(BR.x, BR.y+YR_ADDER, BR.z+(BR.y+YR_ADDER)*ZR_MULTIPLIER);
        backLeftLeg.addPoint(BL.x, BL.y+YL_ADDER, BL.z+(BL.y+YL_ADDER)*ZL_MULTIPLIER);
    }
/*
    public void transferWeight(double direction) {
        //-1 is back 1 is forwards and 0 is nothing.
        if (direction == 1.0 || direction == 0.0 || direction == -1.0) {
            FL = frontLeftLeg.getFootPosition();
            FR = frontRightLeg.getFootPosition();
            BR = backRightLeg.getFootPosition();
            BL = backLeftLeg.getFootPosition();

            double mag = 1.0/50.0;

            frontLeftLeg.addPoint(FL.x+direction*mag, FL.y, FL.z);
            frontRightLeg.addPoint(FR.x+direction*mag, FR.y, FR.z);
            backRightLeg.addPoint(BR.x+direction*mag, BR.y, BR.z);
            backLeftLeg.addPoint(BL.x+direction*mag, BL.y, BL.z);
        }
    }
    */

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