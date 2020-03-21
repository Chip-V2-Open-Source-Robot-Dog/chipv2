package edu.mit.chip.robotmovement;

import edu.mit.chip.mechanisms.Leg;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryGenerator {
    Leg frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;

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