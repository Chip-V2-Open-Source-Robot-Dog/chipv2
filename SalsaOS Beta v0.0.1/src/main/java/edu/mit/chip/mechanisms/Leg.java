package edu.mit.chip.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.ArrayList;

public class Leg {

    double L1;
    double L2;
    double L3;
    double L4;
    double L5;
    double L6;

    double shoulderMultiplier = 1.0;
    double hingeMultiplier = 1.0;
    double kneeMultiplier = 1.0; 

    public double[] home;

    public enum JointType {
        SHOULDER,
        HINGE,
        KNEE;
    }

    public enum MotorControlType {
        VOLTAGE(ControlType.kVoltage),
        VELOCITY(ControlType.kVelocity),
        POSITION(ControlType.kPosition);

        public ControlType sparkMaxType;

        private MotorControlType(ControlType sparkMaxType) {
            this.sparkMaxType = sparkMaxType;
        }
    }

    public CANSparkMax shoulder, hinge, knee;

    public Leg(int shoulderID, int hingeID, int kneeID, double[] model, boolean shoulder_rev, boolean hinge_rev, boolean knee_rev) {
        shoulder =  new CANSparkMax(shoulderID, MotorType.kBrushless);
        hinge =     new CANSparkMax(hingeID,    MotorType.kBrushless);
        knee =      new CANSparkMax(kneeID,     MotorType.kBrushless);

        if(model.length==6) {
            L1 = model[0];
            L2 = model[1];
            L3 = model[2];
            L4 = model[3];
            L5 = model[4];
            L6 = model[5];
        }
        else {
            throw new IllegalArgumentException("WARNING: Leg model does not match internal model. Too many or too few arguments!");
        }

        if(shoulder_rev) {
            shoulderMultiplier = -1.0;
        }
        if(hinge_rev) {
            hingeMultiplier = -1.0;
        }
        if(knee_rev) {
            kneeMultiplier = -1.0;
        }

        home = forwardsKinematics(new double[]{0.0,0.0,0.0});
    }

    public void loadPID(PIDConstants shoulderPID, PIDConstants hingePID, PIDConstants kneePID) {
        shoulderPID.load(shoulder.getPIDController());
        hingePID.load(hinge.getPIDController());
        kneePID.load(knee.getPIDController());
    }

    public void set(MotorControlType controlType, double shoulderVal, double hingeVal, double kneeVal) {
        shoulder.getPIDController().setReference(shoulderVal, controlType.sparkMaxType);
        hinge.getPIDController().setReference(hingeVal, controlType.sparkMaxType);
        knee.getPIDController().setReference(kneeVal, controlType.sparkMaxType);
    }

    public void neutral() {
        set(MotorControlType.VOLTAGE, 0, 0, 0);
    }

    public void set(LegPosition legPosition) {
        set(MotorControlType.POSITION, legPosition.shoulder, legPosition.hinge, legPosition.knee);
    }

    public LegPosition getPosition() {
        return new LegPosition(getPosition(JointType.SHOULDER), getPosition(JointType.HINGE), getPosition(JointType.KNEE));
    }

    public void setEncoders(LegPosition legPosition) {
        shoulder.getEncoder().setPosition(legPosition.shoulder);
        hinge.getEncoder().setPosition(legPosition.hinge);
        knee.getEncoder().setPosition(legPosition.knee);
    }

    public double getPosition(JointType joint) {
        switch (joint) {
            case SHOULDER:
                return shoulder.getEncoder().getPosition();
            case HINGE:
                return hinge.getEncoder().getPosition();
            case KNEE:
                return knee.getEncoder().getPosition();
        }
        return 0;
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

    /*
    FORWARDS KINEMATICS AND CONTROL CODE
    */
    public double[] getThetas() {
        double theta_1 = getPosition(JointType.SHOULDER)*2*Math.PI/100.0;
        double theta_2 = getPosition(JointType.HINGE)*2*Math.PI/100.0;
        double theta_3 = getPosition(JointType.KNEE)*2*Math.PI/100.0;
        return new double[]{theta_1, theta_2, theta_3};
    }

    public double[] forwardsKinematics(double[] thetas) {
        double theta_1 = thetas[0];
        double theta_2 = thetas[1];
        double theta_3 = thetas[2];

        //IMPORTED MATLAB MODEL
        double xe =  L5*(Math.cos(theta_1)*Math.sin(theta_3) + Math.cos(theta_2)*Math.cos(theta_3)*Math.sin(theta_1)) - L6*(Math.cos(theta_1)*Math.cos(theta_3) - Math.cos(theta_2)*Math.sin(theta_1)*Math.sin(theta_3)) + L3*Math.cos(theta_1) + L2*Math.sin(theta_1) + L4*Math.sin(theta_1)*Math.sin(theta_2);
        double ye =  L5*(Math.sin(theta_1)*Math.sin(theta_3) - Math.cos(theta_1)*Math.cos(theta_2)*Math.cos(theta_3)) - L6*(Math.cos(theta_3)*Math.sin(theta_1) + Math.cos(theta_1)*Math.cos(theta_2)*Math.sin(theta_3)) - L2*Math.cos(theta_1) + L3*Math.sin(theta_1) - L4*Math.cos(theta_1)*Math.sin(theta_2);
        double ze =  L1 + L4*Math.cos(theta_2) - L5*Math.cos(theta_3)*Math.sin(theta_2) - L6*Math.sin(theta_2)*Math.sin(theta_3);

        return new double[]{xe, ye, ze};
    }
    /*
    END CONTROL CODE
    */
    public void updateDashboard(String name) {
        SmartDashboard.putNumber(name + " - Shoulder Velocity", getVelocity(JointType.SHOULDER));
        SmartDashboard.putNumber(name + " - Hinge Velocity", getVelocity(JointType.HINGE));
        SmartDashboard.putNumber(name + " - Knee Velocity", getVelocity(JointType.KNEE));

        SmartDashboard.putNumber(name + " - Shoulder Position", getPosition(JointType.SHOULDER));
        SmartDashboard.putNumber(name + " - Hinge Position", getPosition(JointType.HINGE));
        SmartDashboard.putNumber(name + " - Knee Position", getPosition(JointType.KNEE));
    }
}