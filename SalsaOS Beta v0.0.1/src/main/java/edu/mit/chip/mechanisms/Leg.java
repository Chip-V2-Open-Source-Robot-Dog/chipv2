package edu.mit.chip.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.leg.FootPosition;
import edu.mit.chip.leg.LegModel;
import edu.mit.chip.leg.LegPosition;
import edu.mit.chip.leg.LegReversal;
import edu.mit.chip.leg.LegThetas;
import edu.mit.chip.utils.Networking;
import edu.mit.chip.utils.PIDConstants;
import edu.mit.chip.utils.RobotMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leg {
    public LegModel model;
    public LegReversal reversal;

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

    public Leg(LegModel model, int shoulderID, boolean shoulderReverse, int hingeID, boolean hingeReverse, int kneeID, boolean kneeReverse) {
        shoulder =  new CANSparkMax(shoulderID, MotorType.kBrushless);
        hinge =     new CANSparkMax(hingeID,    MotorType.kBrushless);
        knee =      new CANSparkMax(kneeID,     MotorType.kBrushless);

        this.model = model;

        reversal = new LegReversal(shoulderReverse, hingeReverse, kneeReverse);
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
        set(MotorControlType.POSITION, reversal.shoulderMult * legPosition.shoulder, reversal.hingeMult * legPosition.hinge, reversal.kneeMult * legPosition.knee);
    }

    public LegPosition getPosition() {
        return new LegPosition(getPosition(JointType.SHOULDER), getPosition(JointType.HINGE), getPosition(JointType.KNEE));
    }

    public LegThetas getThetas() {
        LegPosition currentPosition = getPosition();
        return new LegThetas(RobotMath.calculateTheta(currentPosition.shoulder), RobotMath.calculateTheta(currentPosition.hinge), RobotMath.calculateTheta(currentPosition.knee));
    }

    public void setEncoders(LegPosition legPosition) {
        shoulder.getEncoder().setPosition(legPosition.shoulder);
        hinge.getEncoder().setPosition(legPosition.hinge);
        knee.getEncoder().setPosition(legPosition.knee);
    }

    public double getPosition(JointType joint) {
        switch (joint) {
            case SHOULDER:
                return reversal.shoulderMult * shoulder.getEncoder().getPosition();
            case HINGE:
                return reversal.hingeMult * hinge.getEncoder().getPosition();
            case KNEE:
                return reversal.kneeMult * knee.getEncoder().getPosition();
        }
        return 0;
    }

    public double getVelocity(JointType joint) {
        switch (joint) {
            case SHOULDER:
                return reversal.shoulderMult * shoulder.getEncoder().getVelocity();
            case HINGE:
                return reversal.hingeMult * hinge.getEncoder().getVelocity();
            case KNEE:
                return reversal.kneeMult * knee.getEncoder().getVelocity();
        }
        return 0;
    }

    //WILL ALSO NEED A CURRENT LEG XYZ POS METHOD (FORWARDS KINEMATICS)
    public FootPosition getFootPosition() {
        double l1 = model.L3; //Math.sqrt(L1*L1+L3*L3);
        double l2 = model.L6; //Math.sqrt(L4*L4+L6*L6);
        
        LegThetas thetas = getThetas();

        double xe = l1*Math.cos(thetas.shoulder)-l2*Math.cos(thetas.knee-thetas.shoulder);
        double ye = l1*Math.sin(thetas.shoulder)+l2*Math.sin(thetas.knee-thetas.shoulder);
        double ze = ye*Math.sin(thetas.hinge);

        return new FootPosition(xe, ye, ze);
    }

    public void updateDashboard(String name) {
        SmartDashboard.putNumber(name + " - Shoulder Velocity", getVelocity(JointType.SHOULDER));
        SmartDashboard.putNumber(name + " - Hinge Velocity", getVelocity(JointType.HINGE));
        SmartDashboard.putNumber(name + " - Knee Velocity", getVelocity(JointType.KNEE));

        SmartDashboard.putNumber(name + " - Shoulder Position", getPosition(JointType.SHOULDER));
        SmartDashboard.putNumber(name + " - Hinge Position", getPosition(JointType.HINGE));
        SmartDashboard.putNumber(name + " - Knee Position", getPosition(JointType.KNEE));
    }

    public void pushData(Networking networking, String prefix) {
        FootPosition footPosition = getFootPosition();
        networking.pushReadout(prefix + "_x", footPosition.x);
        networking.pushReadout(prefix + "_y", footPosition.y);
        networking.pushReadout(prefix + "_z", footPosition.z);

        LegThetas thetas = getThetas();
        networking.pushReadout(prefix + "_shoulderTheta", thetas.shoulder);
        networking.pushReadout(prefix + "_hingeTheta",    thetas.hinge);
        networking.pushReadout(prefix + "_kneeTheta",     thetas.knee);

        double current = Math.abs(shoulder.getOutputCurrent()) + Math.abs(hinge.getOutputCurrent()) + Math.abs(knee.getOutputCurrent());
        networking.pushReadout(prefix + "_current", current);
    }
}