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

    public enum ChipControlType {
        VELOCITY = ControlType.kVelocity,
        POSITION = ControlType.kPosition;
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

    public void setReferences(double shoulderVal, double hingeVal, double kneeVal, ChipControlType type) {
        shoulder.getPIDController().setReference(shoulderVal, type);
        hinge.getPIDController().setReference(hingeVal, type);
        knee.getPIDController().setReference(kneeVal, type);
    }

    public double getEncoderVals(JointType joint, ChipControlType type) {
        switch(type){
            case VELOCITY:
                switch (joint) {
                    case SHOULDER:
                        return shoulder.getEncoder().getVelocity();
                    case HINGE:
                        return hinge.getEncoder().getVelocity();
                    case KNEE:
                        return knee.getEncoder().getVelocity();
                }
                break;
            case POSITION:
                switch (joint) {
                    case SHOULDER:
                        return shoulder.getEncoder().getPosition();
                    case HINGE:
                        return hinge.getEncoder().getPosition();
                    case KNEE:
                        return knee.getEncoder().getPosition();
                }
                break;
                //We may want to at some point convert position to our reference angles?
        }
        return 0;
    }

    public void updateDashboard(String name) {
        SmartDashboard.putNumber(name + " - Shoulder Velocity", getEncoderVals(JointType.SHOULDER, ChipControlType.VELOCITY));
        SmartDashboard.putNumber(name + " - Hinge Velocity", getEncoderVals(JointType.HINGE, ChipControlType.VELOCITY));
        SmartDashboard.putNumber(name + " - Knee Velocity", getEncoderVals(JointType.KNEE, ChipControlType.VELOCITY));

        SmartDashboard.putNumber(name + " - Shoulder Position", getEncoderVals(JointType.SHOULDER, ChipControlType.POSITION));
        SmartDashboard.putNumber(name + " - Hinge Position", getEncoderVals(JointType.HINGE, ChipControlType.POSITION));
        SmartDashboard.putNumber(name + " - Knee Position", getEncoderVals(JointType.KNEE, ChipControlType.POSITION));
    }
}