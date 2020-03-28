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
import edu.mit.chip.setpoint.SetpointManager;
import edu.mit.chip.leg.FootPosition;
import edu.mit.chip.leg.LegModel;
import edu.mit.chip.leg.LegType;
import edu.mit.chip.utils.Networking;
import edu.mit.chip.utils.PIDConstants;
import edu.mit.chip.utils.SpeedSet;

import edu.wpi.first.wpilibj.TimedRobot;
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

    private final double kMaxOutput = 1.0;
    private final double kMinOutput = -1.0;
    private final double maxRPM = 5700;

    // private TrajectoryRunner trajectoryRunner;

    protected final Leg[] legs = {frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg};
    protected final LegType[] legTypes = {LegType.FRONT_LEFT, LegType.FRONT_RIGHT, LegType.BACK_LEFT, LegType.BACK_RIGHT};

    protected final FootPosition defaultFootPosition = new FootPosition(0, 0.15, 0);
    protected SetpointManager setpointManager;

    Joystick joy = new Joystick(0);

    private Networking networking;
    // private Thread networkingThread;

    /**
     * This function is run when the robot code is first started up (or restarted).
     */
    @Override
    public void robotInit() {
        System.out.println("Initializing robot...");
        System.out.println("Attempting to construct legs.");

        LegModel leftLegs = new LegModel(0.055, 0.075, 0.235, 0.1, 0.03, 0.32);
        LegModel rightLegs = new LegModel(0.055, -0.075, 0.235, 0.1, -0.03, 0.32);

        frontLeftLeg = new Leg(leftLegs, 3, true, 2, false, 1, false);
        frontRightLeg = new Leg(rightLegs, 12, false, 10, true, 11, true);
        backLeftLeg = new Leg(leftLegs, 4, true, 6, false, 5, false);
        backRightLeg = new Leg(rightLegs, 9, false, 7, true, 8, true);

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

        // trajectoryRunner = new TrajectoryRunner(this, new SpeedSet(0.5, 0.5, 0.5, 0.5));

        networking = Networking.getInstance();
        for (LegType legType : legTypes) {
            networking.addReadouts(
                legType.key("x"), legType.key("y"), legType.key("z"),
                legType.key("shoulderTheta"), legType.key("hingeTheta"), legType.key("kneeTheta"),
                legType.key("current")
            );
            networking.addInputs(
                legType.key("x"), legType.key("y"), legType.key("z")
            );
        }
    }
    
    /**
    * This function is called every robot packet, no matter the mode.
    */
    @Override
    public void robotPeriodic() {
        for (LegType legType : legTypes) {
            Leg leg = getLeg(legType);

            leg.updateDashboard(legType.name);
            leg.pushData(networking, legType.prefix);
        }
    }
    
    /**
    * This function is called at the very beginning of the teleoperated period.
    */
    @Override
    public void teleopInit() {
        // trajectoryRunner.reset();
        setpointManager = new SetpointManager(this, new SpeedSet(0.5, 0.5, 0.5, 0.5));
        for (LegType legType : legTypes) {
            networking.initInput(legType.key("x"), defaultFootPosition.x);
            networking.initInput(legType.key("y"), defaultFootPosition.y);
            networking.initInput(legType.key("z"), defaultFootPosition.z);
        }
        pullSetpoint();
        setpointManager.resume();
    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {
        // trajectoryRunner.tick();

        pullSetpoint();
        setpointManager.tick();
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

    public Leg getLeg(LegType type) {
        if (type == LegType.FRONT_LEFT) {
            return frontLeftLeg;
        }
        else if (type == LegType.FRONT_RIGHT) {
            return frontRightLeg;
        }
        else if (type == LegType.BACK_LEFT) {
            return backLeftLeg;
        }
        else {
            return backRightLeg;
        }
    }

    public void pullSetpoint() {
        for (LegType legType : legTypes) {
            this.setpointManager.updateSetpoint(legType,
                networking.pullInput(legType.prefix + "_x", this.defaultFootPosition.x),
                networking.pullInput(legType.prefix + "_y", this.defaultFootPosition.y),
                networking.pullInput(legType.prefix + "_z", this.defaultFootPosition.z)
            );
        }
    }
}
