/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.moveactions.CmdMoveRobot;
import edu.mit.chip.setupactions.SetupActionChooser;
import edu.mit.chip.setupactions.ZeroLegAction;
import edu.mit.chip.trajectory.SpeedSet;
import edu.mit.chip.trajectory.Trajectory;
import edu.mit.chip.trajectory.Waypoint;
import edu.mit.chip.utils.FootPosition;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.PIDConstants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    
    private Trajectory trajectory;

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
        frontRightLeg = new Leg(12, 10, 11, rightLeg, false, true, true);
        backLeftLeg   = new Leg(4, 5, 6, leftLeg, true, false, false);
        backRightLeg  = new Leg(9, 7, 8, rightLeg, false, true, true);
        
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

        trajectory = new Trajectory(this, new SpeedSet(0.5, 0.5, 0.5, 0.5));
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
        // frontLeftLeg.clearTrajectory();
        // frontRightLeg.clearTrajectory();
        // backLeftLeg.clearTrajectory();
        // backRightLeg.clearTrajectory();

        /*
        frontRightLeg.addPoint(0.0, 0.40, 0.0);
        backLeftLeg.addPoint(0.0, 0.45, 0.0);
        backRightLeg.addPoint(0.0, 0.35, 0.0);
        */

        //PRETTY GOOD "STAND UP"
        
        // frontLeftLeg.addPoint(0.0, 0.2, 0.0);
        // frontRightLeg.addPoint(0.0, 0.2, 0.0);
        // backLeftLeg.addPoint(0.05, 0.2, 0.0);
        // backRightLeg.addPoint(0.05, 0.2, 0.0);
        
        // frontLeftLeg.addPoint(0.0, 0.47, 0.0);
        // frontRightLeg.addPoint(0.0, 0.47, 0.0);
        // backLeftLeg.addPoint(0.05, 0.5, 0.0);
        // backRightLeg.addPoint(0.05, 0.5, 0.0);

        // maybe works stand up
        trajectory.addWaypoint(new Waypoint(
            new FootPosition(0.0,  0.47, 0.0),
            new FootPosition(0.0,  0.47, 0.0),
            new FootPosition(0.05, 0.5,  0.0),
            new FootPosition(0.05, 0.5,  0.0)
        ));

        trajectory.addWaypoint(new Waypoint(
            new FootPosition(0.0,  0.2, 0.0),
            new FootPosition(0.0,  0.2, 0.0),
            new FootPosition(0.05, 0.2, 0.0),
            new FootPosition(0.05, 0.2, 0.0)
        ));        
    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {
        trajectory.tick()

        // frontLeftLeg.move(0.5);
        // frontRightLeg.move(0.5);
        // backLeftLeg.move(0.5);
        // backRightLeg.move(0.5);

        /*
        frontLeftLeg.home(0.5);
        frontRightLeg.home(0.5);
        backLeftLeg.home(0.5);
        backRightLeg.home(0.5);
        */
    }
    
    /**
    * This fetches which SetupAction the user has selected and places it in the 
    * CommandQueue.
    */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(setupActionChooser.getChosenActionCmd());
    }
    
    /**
    * This function is called periodically during autonomous. We use it to run
    * SetupAction commands.
    */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();

        frontLeftLeg.neutral();
        frontRightLeg.neutral();
        backLeftLeg.neutral();
        backRightLeg.neutral();
    }
}
