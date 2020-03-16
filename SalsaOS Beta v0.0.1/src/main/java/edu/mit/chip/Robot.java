/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.setupactions.SetupActionChooser;
import edu.mit.chip.setupactions.ZeroLegAction;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.PIDConstants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

import edu.wpi.first.wpilibj.Joystick;

/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the TimedRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public Leg frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;

    private SetupActionChooser setupActionChooser;
    
    private final double kP = 0.1;
    private final double kI = 0.0005;
    private final double kD = 0.01;
    
    private final double kIz = 0;
    private final double kFF = 0;
    
    private final double kMaxOutput =  1.0;
    private final double kMinOutput = -1.0;
    private final double maxRPM = 5700;
    
    private LegPosition frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition;

    Joystick joy = new Joystick(0);
    
    /**
    * This function is run when the robot code is first started up (or restarted).
    */
    @Override
    public void robotInit() {        
        System.out.println("Initializing robot...");
        System.out.println("Attempting to construct legs.");

        double[] leftLeg = new double[]{0.055, 0.075, 0.235, 0.1, 0.03, 0.32};
        double[] rightLeg = new double[]{0.055, -0.075, 0.235, 0.1, -0.03, 0.32};
                
        frontLeftLeg  = new Leg(3, 2, 1, leftLeg, true, false, false);
        frontRightLeg = new Leg(12, 10, 11, rightLeg, true, false, false);
        backLeftLeg   = new Leg(4, 5, 6, leftLeg, false, false, true);
        backRightLeg  = new Leg(9, 7, 8, rightLeg, false, false, true);
        
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

        setupActionChooser = new SetupActionChooser(
            new ZeroLegAction(frontLeftLeg, "Front Left"),
            new ZeroLegAction(frontRightLeg, "Front Right"),
            new ZeroLegAction(backLeftLeg, "Back Left"),
            new ZeroLegAction(backRightLeg, "Back Right")
        );
        setupActionChooser.putOnDashboard();
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

    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {
        // Get value of right joy y axis: joy.getRawAxis(5);
        double speedFactor = 2.0;
        double incrementOne = -speedFactor*joy.getRawAxis(1);
        double incrementTwo = speedFactor*joy.getRawAxis(5);
        //double incrementThree = joy.getRawAxis(4);

        //Josytick BUFFER
        if(Math.abs(incrementOne) <= 0.1){
            incrementOne = 0.0;
        }
        if(Math.abs(incrementTwo) <= 0.1){
            incrementTwo = 0.0;
        }
        /*
        if(Math.abs(incrementThree) <= 0.1){
            incrementTwo = 0.0;
        }
        */

        frontLeftPosition  = frontLeftLeg.getPosition();
        frontRightPosition = frontRightLeg.getPosition();
        backLeftPosition   = backLeftLeg.getPosition();
        backRightPosition  = backRightLeg.getPosition();

        frontLeftPosition.shoulder = frontLeftPosition.shoulder+incrementOne;
        frontLeftPosition.knee = frontLeftPosition.knee-2.0*incrementTwo;
        //frontLeftPosition.hinge = frontLeftPosition.hinge+incrementThree/4.0;

        frontRightPosition.shoulder = frontRightPosition.shoulder-incrementOne;
        frontRightPosition.knee = frontRightPosition.knee+2.0*incrementTwo;
        //frontRightPosition.hinge = frontRightPosition.hinge+incrementThree/4.0;

        backLeftPosition.shoulder = backLeftPosition.shoulder+incrementOne;
        backLeftPosition.knee = backLeftPosition.knee-2.0*incrementTwo;
        //backLeftPosition.hinge = backLeftPosition.hinge+incrementThree/4.0;

        backRightPosition.shoulder = backRightPosition.shoulder-incrementOne;
        backRightPosition.knee = backRightPosition.knee+2.0*incrementTwo;
        //backRightPosition.hinge = backRightPosition.hinge+incrementThree/4.0;

        frontLeftLeg.set(frontLeftPosition);
        frontRightLeg.set(frontRightPosition);
        backLeftLeg.set(backLeftPosition);
        backRightLeg.set(backRightPosition);
    }
    
    /**
    * This fetches which SetupAction the user has selected and places it in the 
    * CommandQueue.
    */
    @Override
    public void autonomousInit() {
        Scheduler.getInstance().add(setupActionChooser.getChosenActionCmd());
    }
    
    /**
    * This function is called periodically during autonomous. We use it to run
    * SetupAction commands.
    */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        frontLeftLeg.neutral();
        frontRightLeg.neutral();
        backLeftLeg.neutral();
        backRightLeg.neutral();

        Scheduler.getInstance().removeAll();
    }
}