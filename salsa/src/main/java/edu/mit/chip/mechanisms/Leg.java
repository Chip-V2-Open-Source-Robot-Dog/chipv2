package edu.mit.chip.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.utils.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leg {
    public enum JointType {
        SHOULDER,
        HINGE,
        KNEE;
    }

    public CANSparkMax shoulder, hinge, knee;

    public Leg(int shoulderID, int hingeID, int kneeID) {
        shoulder =  new CANSparkMax(shoulderID, MotorType.kBrushless);
        hinge =     new CANSparkMax(hingeID,    MotorType.kBrushless);
        knee =      new CANSparkMax(kneeID,     MotorType.kBrushless);
    }

    public void loadPID(PIDConstants shoulderPID, PIDConstants hingePID, PIDConstants kneePID) {
        shoulderPID.load(shoulder.getPIDController());
        hingePID.load(hinge.getPIDController());
        kneePID.load(knee.getPIDController());
    }

    public void setVelocities(double shoulderVel, double hingeVel, double kneeVel) {
        shoulder.getPIDController().setReference(shoulderVel, ControlType.kVelocity);
        hinge.getPIDController().setReference(hingeVel, ControlType.kVelocity);
        knee.getPIDController().setReference(kneeVel, ControlType.kVelocity);
    }

    public double getVelocity(JointType joint) {
        switch (joint) {
            case SHOULDER:
                return shoulder.getEncoder().getVelocity();
            case HINGE:
                return hinge.getEncoder().getVelocity();
            case KNEE:
                return knee.getEncoder().getVelocity();
        }

        return 0;
    }

    public void updateDashboard(String name) {
        SmartDashboard.putNumber(name + " - Shoulder Velocity", getVelocity(JointType.SHOULDER));
        SmartDashboard.putNumber(name + " - Hinge Velocity", getVelocity(JointType.HINGE));
        SmartDashboard.putNumber(name + " - Knee Velocity", getVelocity(JointType.KNEE));
    }
}