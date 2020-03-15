package edu.mit.chip.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public class Leg {
    public enum JointType {
        SHOULDER,
        HINGE,
        KNEE;
    }

    /*
    ADI EDITED THIS PART IT MUST BE FIXED
    */
    double L1;
    double L2;
    double L3;
    double L4; 
    double L5;
    double L6;
    /*
    END EDITING
    */

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

    /*
    ADI EDITED THIS AND IT NEEDS TO BE FIXED
    */
    public Leg(int shoulderID, int hingeID, int kneeID, double[] model) {
        shoulder =  new CANSparkMax(shoulderID, MotorType.kBrushless);
        hinge =     new CANSparkMax(hingeID,    MotorType.kBrushless);
        knee =      new CANSparkMax(kneeID,     MotorType.kBrushless);
        if(model.length == 6) {
            L1 = model[0];
            L2 = model[1];
            L3 = model[2];
            L4 = model[3];
            L5 = model[4];
            L6 = model[5];
        }
        else {
            throw new IllegalArgumentException("WARNING: Leg model given does not have the required number of input parameters, or the leg model is not for the ChipV2.0 Robot. Please check leg model.");
        }
    }
    /*
    END EDITING
    */

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

    /*
    ADI EDITED THIS AND IT NEEDS TO BE FIXED
    */
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

    public List<Double> inverseKinematics(double xDesired, double yDesired, double zDesired){
        double rSquared = xDesired*xDesired+yDesired*yDesired;
        double phi = Math.atan(yDesired/xDesired);
        double theta2 = Math.acos((rSquared-L6*L6-L3*L3)/(-2*L3*L6));
        double alpha = Math.acos((L6*L6-L3*L3-rSquared)/(-2*L3*Math.sqrt(rSquared)));
        double theta1 = phi-alpha;
        double theta3 = 0; //WE WILL CHANGE THIS LATER
        //WE WILL ADD zDesired LATER
        List<Double> thetas = new ArrayList<>(Arrays.asList(new Double(theta1), new Double(theta2), new Double(theta3)));
        //thetas = {theta1, theta2, theta3};
        return thetas;
    }

    public List<Double> convertThetasToCmds(List<Double> thetas){
        List<Double> cmds = new ArrayList<>();
        for(double i : thetas){
            cmds.add(i*100.0/(2.0*Math.PI));
        }
        return cmds;
    }

    public void moveLeg(double xDesired, double yDesired, double zDesired){
        List<Double> thetas = inverseKinematics(xDesired, yDesired, zDesired);
        List<Double> cmds = convertThetasToCmds(thetas);
        set(MotorControlType.POSITION, cmds.get(0), cmds.get(2), -cmds.get(1));
    }
    /*
    END EDITING
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