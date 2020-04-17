/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    //GET CONTROL TYPE ENUM
    public enum MotorControlType {
        VOLTAGE(ControlType.kVoltage),
        VELOCITY(ControlType.kVelocity),
        POSITION(ControlType.kPosition);

        public ControlType sparkMaxType;

        private MotorControlType(ControlType sparkMaxType) {
            this.sparkMaxType = sparkMaxType;
        }
    }

    /*
    FIRST LETS CREATE ALL THE KPS AND ETC
    */
    //COMMON
    private final double kIz = 0;
    private final double kFF = 0;
    private final double maxRPM = 5700;
    //SHOULDER CONSTANTS
    private final double kP_S = 0.05;
    private final double kI_S = 0.0;
    private final double kD_S = 0.0;
    private final double kMaxOutput_S = 0.23;
    private final double kMinOutput_S = -0.23;
    //HINGE CONSTANTS
    private final double kP_H = 0.05;
    private final double kI_H = 0.0;
    private final double kD_H = 0.0;
    private final double kMaxOutput_H = 0.23;
    private final double kMinOutput_H = -0.23;
    //KNEE CONSTANTS
    private final double kP_K = 0.05;
    private final double kI_K = 0.0;
    private final double kD_K = 0.0;
    private final double kMaxOutput_K = 0.23;
    private final double kMinOutput_K = -0.23;

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
        fl_shoulder.setInverted(true);
        //FRONT RIGHT LEG
        fr_shoulder =  new CANSparkMax(12, MotorType.kBrushless);
        fr_hinge =     new CANSparkMax(10, MotorType.kBrushless);
        fr_knee =      new CANSparkMax(11, MotorType.kBrushless);
        fr_hinge.setInverted(true);
        fr_knee.setInverted(true);
        //BACK LEFT LEG
        bl_shoulder =  new CANSparkMax(4, MotorType.kBrushless);
        bl_hinge =     new CANSparkMax(6, MotorType.kBrushless);
        bl_knee =      new CANSparkMax(5, MotorType.kBrushless);
        bl_shoulder.setInverted(true);
        //BACK RIGHT LEG
        br_shoulder =  new CANSparkMax(9, MotorType.kBrushless);
        br_hinge =     new CANSparkMax(7, MotorType.kBrushless);
        br_knee =      new CANSparkMax(8, MotorType.kBrushless);
        br_hinge.setInverted(true);
        br_knee.setInverted(true);

        System.out.println("Controllers constructed.");

        //CREATE PID CONSTANTS OBJECT
        PIDConstants shoulderPID = new PIDConstants(kP_S, kI_S, kD_S, kIz, kFF, kMaxOutput_S, kMinOutput_S, maxRPM);
        PIDConstants hingePID = new PIDConstants(kP_H, kI_H, kD_H, kIz, kFF, kMaxOutput_H, kMinOutput_H, maxRPM);
        PIDConstants kneePID = new PIDConstants(kP_K, kI_K, kD_K, kIz, kFF, kMaxOutput_K, kMinOutput_K, maxRPM);
        //actually set the PID constants
        shoulderPID.load(fl_shoulder.getPIDController());
        hingePID.load(fl_hinge.getPIDController());
        kneePID.load(fl_knee.getPIDController());

        shoulderPID.load(fr_shoulder.getPIDController());
        hingePID.load(fr_hinge.getPIDController());
        kneePID.load(fr_knee.getPIDController());

        shoulderPID.load(bl_shoulder.getPIDController());
        hingePID.load(bl_hinge.getPIDController());
        kneePID.load(bl_knee.getPIDController());

        shoulderPID.load(br_shoulder.getPIDController());
        hingePID.load(br_hinge.getPIDController());
        kneePID.load(br_knee.getPIDController());

        System.out.println("Robot initialized.");

        //DO NETWORKING SETUP
        networking = Networking.getInstance();
        networking.addReadouts(
            "fl_s", "fl_h", "fl_k", "fl_s_current", "fl_h_current", "fl_k_current",
            "fr_s", "fr_h", "fr_k", "fr_s_current", "fr_h_current", "fr_k_current",
            "bl_s", "bl_h", "bl_k", "bl_s_current", "bl_h_current", "bl_k_current", 
            "br_s", "br_h", "br_k", "br_s_current", "br_h_current", "br_k_current",
            "VBUS"
        );
        networking.addInputs(
            "fl_s", "fl_h", "fl_k",
            "fr_s", "fr_h", "fr_k",
            "bl_s", "bl_h", "bl_k",
            "br_s", "br_h", "br_k"
        );

        //ENABLE THE ROBOT WITH DS
        DS.start();
    }
    
    /**
    * This function is called every robot packet, no matter the mode.
    */
    @Override
    public void robotPeriodic() {
        //PERIODICALLY PUBLISH VALUES
        //FRONT LEFT LEG
        networking.pushReadout("fl_s", fl_shoulder.getEncoder().getPosition());
        networking.pushReadout("fl_h", fl_hinge.getEncoder().getPosition());
        networking.pushReadout("fl_k", fl_knee.getEncoder().getPosition());
        networking.pushReadout("fl_s_current", fl_shoulder.getOutputCurrent());
        networking.pushReadout("fl_h_current", fl_hinge.getOutputCurrent());
        networking.pushReadout("fl_k_current", fl_knee.getOutputCurrent());
        
        //FRONT RIGHT LEG
        networking.pushReadout("fr_s", fr_shoulder.getEncoder().getPosition());
        networking.pushReadout("fr_h", fr_hinge.getEncoder().getPosition());
        networking.pushReadout("fr_k", fr_knee.getEncoder().getPosition());
        networking.pushReadout("fr_s_current", fr_shoulder.getOutputCurrent());
        networking.pushReadout("fr_h_current", fr_hinge.getOutputCurrent());
        networking.pushReadout("fr_k_current", fr_knee.getOutputCurrent());

        //BACK LEFT LEG
        networking.pushReadout("bl_s", bl_shoulder.getEncoder().getPosition());
        networking.pushReadout("bl_h", bl_hinge.getEncoder().getPosition());
        networking.pushReadout("bl_k", bl_knee.getEncoder().getPosition());
        networking.pushReadout("bl_s_current", bl_shoulder.getOutputCurrent());
        networking.pushReadout("bl_h_current", bl_hinge.getOutputCurrent());
        networking.pushReadout("bl_k_current", bl_knee.getOutputCurrent());

        //BACK RIGHT LEG
        networking.pushReadout("br_s", br_shoulder.getEncoder().getPosition());
        networking.pushReadout("br_h", br_hinge.getEncoder().getPosition());
        networking.pushReadout("br_k", br_knee.getEncoder().getPosition());
        networking.pushReadout("br_s_current", br_shoulder.getOutputCurrent());
        networking.pushReadout("br_h_current", br_hinge.getOutputCurrent());
        networking.pushReadout("br_k_current", br_knee.getOutputCurrent());

        //CALCULATE AVERAGE BUS VOLTAGE
        double AVG_VOLTAGE = (fl_shoulder.getBusVoltage()+fr_shoulder.getBusVoltage()+bl_shoulder.getBusVoltage()+br_shoulder.getBusVoltage()+
                            fl_hinge.getBusVoltage()+fr_hinge.getBusVoltage()+bl_hinge.getBusVoltage()+br_hinge.getBusVoltage()+
                            fl_knee.getBusVoltage()+fr_knee.getBusVoltage()+bl_knee.getBusVoltage()+br_knee.getBusVoltage())/12.0;
        //PUSH AVERAGE BUS VOLTAGE
        networking.pushReadout("VBUS", AVG_VOLTAGE);
    }
    
    /**
    * This function is called at the very beginning of the teleoperated period.
    */
    @Override
    public void teleopInit() {
        //SET ALL THE LEGS TO ZERO POSITION 
        fl_shoulder.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        fl_hinge.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        fl_knee.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);

        fr_shoulder.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        fr_hinge.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        fr_knee.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);

        bl_shoulder.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        bl_hinge.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        bl_knee.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);

        br_shoulder.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        br_hinge.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
        br_knee.getPIDController().setReference(0.0, MotorControlType.POSITION.sparkMaxType);
    }
    
    /**
    * This function is called periodically during teleoperated period.
    */
    @Override
    public void teleopPeriodic() {
        //SET THE VALUES TO WHAT IS PULLED FROM NETWORK TABLES 
        fl_shoulder.getPIDController().setReference(networking.pullInput("fl_s", 0.0), MotorControlType.POSITION.sparkMaxType);
        fl_hinge.getPIDController().setReference(networking.pullInput("fl_h", 0.0), MotorControlType.POSITION.sparkMaxType);
        fl_knee.getPIDController().setReference(networking.pullInput("fl_k", 0.0), MotorControlType.POSITION.sparkMaxType);

        fr_shoulder.getPIDController().setReference(networking.pullInput("fr_s", 0.0), MotorControlType.POSITION.sparkMaxType);
        fr_hinge.getPIDController().setReference(networking.pullInput("fr_h", 0.0), MotorControlType.POSITION.sparkMaxType);
        fr_knee.getPIDController().setReference(networking.pullInput("fr_k", 0.0), MotorControlType.POSITION.sparkMaxType);

        bl_shoulder.getPIDController().setReference(networking.pullInput("bl_s", 0.0), MotorControlType.POSITION.sparkMaxType);
        bl_hinge.getPIDController().setReference(networking.pullInput("bl_h", 0.0), MotorControlType.POSITION.sparkMaxType);
        bl_knee.getPIDController().setReference(networking.pullInput("bl_k", 0.0), MotorControlType.POSITION.sparkMaxType);

        br_shoulder.getPIDController().setReference(networking.pullInput("br_s", 0.0), MotorControlType.POSITION.sparkMaxType);
        br_hinge.getPIDController().setReference(networking.pullInput("br_h", 0.0), MotorControlType.POSITION.sparkMaxType);
        br_knee.getPIDController().setReference(networking.pullInput("br_k", 0.0), MotorControlType.POSITION.sparkMaxType);
    }
    
    /**
    * This fetches which SetupAction the user has selected and places it in the 
    * CommandQueue.
    */
    @Override
    public void autonomousInit() {
        //DO NOTHING
    }
    
    /**
    * This function is called periodically during autonomous. We use it to run
    * SetupAction commands.
    */
    @Override
    public void autonomousPeriodic() {
        //DO NOTHING
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        //SET ALL THE MOTORS TO NEUTRAL/VOLTAGE CONTROL and 0 VOLTS
        fl_shoulder.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        fl_hinge.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        fl_knee.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);

        fr_shoulder.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        fr_hinge.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        fr_knee.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);

        bl_shoulder.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        bl_hinge.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        bl_knee.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);

        br_shoulder.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        br_hinge.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
        br_knee.getPIDController().setReference(0.0, MotorControlType.VOLTAGE.sparkMaxType);
    }
}
