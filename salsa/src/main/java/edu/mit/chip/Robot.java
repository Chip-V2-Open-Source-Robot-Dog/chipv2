/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mit.chip;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.mechanisms.Leg.JointType;
import edu.mit.chip.utils.PIDConstants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the TimedRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public Leg frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;
    
    private final double kP = 0.001;
    private final double kI = 0;
    private final double kD = 0;
    
    private final double kIz = 0;
    private final double kFF = 0;
    
    private final double kMaxOutput =  1.0;
    private final double kMinOutput = -1.0;
    private final double maxRPM = 5700;
    
    /**
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
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
    * This function is called every robot packet, no matter the mode. Use
    * this for items like diagnostics that you want ran during disabled,
    * autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        frontLeftLeg.updateDashboard("Front Left");
        frontRightLeg.updateDashboard("Front Right");

        backLeftLeg.updateDashboard("Back Left");
        backRightLeg.updateDashboard("Back Right");
    }
    
    /**
    * This autonomous (along with the chooser code above) shows how to select
    * between different autonomous modes using the dashboard. The sendable
    * chooser code works with the Java SmartDashboard. If you prefer the
    * LabVIEW Dashboard, remove all of the chooser code and uncomment the
    * getString line to get the auto name from the text box below the Gyro
    *
    * <p>You can add additional auto modes by adding additional comparisons to
    * the switch structure below with additional strings. If using the
    * SendableChooser make sure to add them to the chooser code above as well.
    */
    @Override
    public void autonomousInit() {
    }
    
    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
    }
    
    /**
    * This function is called periodically during operator control.
    */
    @Override
    public void teleopPeriodic() {
        frontLeftLeg.setVelocities(0, 0, 0);
        frontRightLeg.setVelocities(0, 0, 0);
        backLeftLeg.setVelocities(0, 0, 0);
        backRightLeg.setVelocities(0, 0, 0);
    }
    
    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }
}
