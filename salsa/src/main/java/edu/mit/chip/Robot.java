/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.PIDConstants;
import edu.wpi.first.wpilibj.TimedRobot;

/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the TimedRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    public Leg frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;
    
    private final double kP = 0.050;
    private final double kI = 0;
    private final double kD = 0;
    
    private final double kIz = 0;
    private final double kFF = 0;
    
    private final double kMaxOutput =  1.0;
    private final double kMinOutput = -1.0;
    private final double maxRPM = 5700;

    private LegPosition frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition;
    
    /**
    * This function is run when the robot code is first started up (or restarted).
    */
    @Override
    public void robotInit() {
        System.out.println("Initializing robot...");
        System.out.println("Attempting to construct legs.");

        frontLeftLeg  = new Leg(3, 2, 1);
        frontRightLeg = new Leg(12, 10, 11);
        backLeftLeg   = new Leg(4, 5, 6);
        backRightLeg  = new Leg(9, 7, 8);

        System.out.println("Legs constructed.");
        
        frontLeftLeg.loadPID(
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM)
        );

        frontRightLeg.loadPID(
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM)
        );
        
        backLeftLeg.loadPID(
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM)
        );

        backRightLeg.loadPID(
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM),
            new PIDConstants(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM)
        );
        
        System.out.println("Robot initialized.");
    }
    
    /**
    * This function is called every robot packet, no matter the mode.
    */
    @Override
    public void robotPeriodic() {
        frontLeftLeg.updateDashboard("Front Left");
        frontRightLeg.updateDashboard("Front Right");

        backLeftLeg.updateDashboard("Back Left");
        backRightLeg.updateDashboard("Back Right");
    }

    /**
     * This function is called at the very beginning of the teleoperated period.
     */
    @Override
    public void teleopInit() {
        frontLeftPosition  = frontLeftLeg.getPosition();
        frontRightPosition = frontRightLeg.getPosition();
        backLeftPosition   = backLeftLeg.getPosition();
        backRightPosition  = backRightLeg.getPosition();
    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {
        frontLeftLeg.set(frontLeftPosition);
        frontRightLeg.set(frontRightPosition);
        backLeftLeg.set(backLeftPosition);
        backRightLeg.set(backRightPosition);
    }
}
