/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import edu.mit.chip.utils.Networking;
import edu.mit.chip.utils.PIDConstants;
import edu.mit.chip.utils.DS;

import edu.wpi.first.wpilibj.TimedRobot;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /*
    FIRST LETS CREATE ALL THE KPS AND ETC
    */
    //COMMON
    private final double kIz = 0;
    private final double kFF = 0;
    private final double maxRPM = 5700;
    //SHOULDER AND HINGE CONSTANTS
    private final double kP_S = 0.2;
    private final double kI_S = 0.0005;
    private final double kD_S = 0.01;
    private final double kMaxOutput_S = 1.0;
    private final double kMinOutput_S = -1.0;
    //KNEE CONSTANTS
    private final double kP_K = 0.2;
    private final double kI_K = 0.0005;
    private final double kD_K = 0.01;
    private final double kMaxOutput_K = 1.0;
    private final double kMinOutput_K = -1.0;

    private Networking networking;
    // private Thread networkingThread;

    private DS DS = new DS();

    //Create SPARKMAX Objects
    private CANSparkMax fl_shoulder, fl_hinge, fl_knee, fr_shoulder, fr_hinge, fr_knee, bl_shoulder, bl_hinge, bl_knee, br_shoulder, br_hinge, br_knee;
    /**
     * This function is run when the robot code is first started up (or restarted).
     */
    @Override
    public void robotInit() {
        System.out.println("Initializing robot...");
        System.out.println("Attempting to construct CAN SparkMAXs.");

        /*
        LegModel leftLegs = new LegModel(0.055, 0.075, 0.235, 0.1, 0.03, 0.32);
        LegModel rightLegs = new LegModel(0.055, -0.075, 0.235, 0.1, -0.03, 0.32);

        frontLeftLeg = new Leg(leftLegs, 3, true, 2, false, 1, false);
        frontRightLeg = new Leg(rightLegs, 12, false, 10, true, 11, true);
        backLeftLeg = new Leg(leftLegs, 4, true, 6, false, 5, false);
        backRightLeg = new Leg(rightLegs, 9, false, 7, true, 8, true);
        */

        //CREATE SPARKMAXS AND DEAL WITH REVERSALS
        //FRONT LEFT LEG
        fl_shoulder =  new CANSparkMax(3, MotorType.kBrushless);
        fl_hinge =     new CANSparkMax(2, MotorType.kBrushless);
        fl_knee =      new CANSparkMax(1, MotorType.kBrushless);
        fl_shoulder.isInverted(true);
        //FRONT RIGHT LEG
        fr_shoulder =  new CANSparkMax(12, MotorType.kBrushless);
        fr_hinge =     new CANSparkMax(10, MotorType.kBrushless);
        fr_knee =      new CANSparkMax(11, MotorType.kBrushless);
        fr_hinge.isInverted(true);
        fr_knee.isInverted(true);
        //BACK LEFT LEG
        bl_shoulder =  new CANSparkMax(4, MotorType.kBrushless);
        bl_hinge =     new CANSparkMax(6, MotorType.kBrushless);
        bl_knee =      new CANSparkMax(5, MotorType.kBrushless);
        bl_shoulder.isInverted(true);
        //BACK RIGHT LEG
        br_shoulder =  new CANSparkMax(9, MotorType.kBrushless);
        br_hinge =     new CANSparkMax(7, MotorType.kBrushless);
        br_knee =      new CANSparkMax(8, MotorType.kBrushless);
        br_hinge.isInverted(true);
        br_knee.isInverted(true);

        System.out.println("Controllers constructed.");

        //CREATE PID CONSTANTS OBJECT
        shoulder_hingePID = new PIDConstants(kP_S, kI_S, kD_S, kIz, kFF, kMaxOutput_S, kMinOutput_S, maxRPM);
        kneePID = new PIDConstants(kP_K, kI_K, kD_K, kIz, kFF, kMaxOutput_K, kMinOutput_K, maxRPM);
        //actually set the PID constants
        shoulder_hingePID.load(fl_shoulder.getPIDController());
        shoulder_hingePID.load(fl_hinge.getPIDController());
        kneePID.load(fl_knee.getPIDController());

        shoulder_hingePID.load(fr_shoulder.getPIDController());
        shoulder_hingePID.load(fr_hinge.getPIDController());
        kneePID.load(fr_knee.getPIDController());

        shoulder_hingePID.load(bl_shoulder.getPIDController());
        shoulder_hingePID.load(bl_hinge.getPIDController());
        kneePID.load(bl_knee.getPIDController());

        shoulder_hingePID.load(br_shoulder.getPIDController());
        shoulder_hingePID.load(br_hinge.getPIDController());
        kneePID.load(br_knee.getPIDController());

        System.out.println("Robot initialized.");

        //DO NETWORKING SETUP
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

        //ENABLE THE ROBOT WITH DS
        DS.start();
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
        setpointManager = new SetpointManager(this, new SpeedSet(0.7, 0.7, 0.7, 0.7));
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
