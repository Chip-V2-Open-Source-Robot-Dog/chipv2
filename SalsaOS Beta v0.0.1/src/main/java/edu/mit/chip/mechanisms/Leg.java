package edu.mit.chip.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.mit.chip.utils.FootPosition;
import edu.mit.chip.utils.LegModel;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.LegReversal;
import edu.mit.chip.utils.LegThetas;
import edu.mit.chip.utils.PIDConstants;
import edu.mit.chip.utils.RobotMath;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Leg {
    public LegModel model;
    public LegReversal reversal;

    private LegPosition position;
    private ArrayList<double[]> trajectory;
    public LegPosition homeCMD = new LegPosition(0.0, 0.0, 0.0);

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

    public Leg(int shoulderID, int hingeID, int kneeID, LegModel model, boolean revShoulder, boolean revHinge, boolean revKnee) {
        shoulder =  new CANSparkMax(shoulderID, MotorType.kBrushless);
        hinge =     new CANSparkMax(hingeID,    MotorType.kBrushless);
        knee =      new CANSparkMax(kneeID,     MotorType.kBrushless);

        this.model = model;

        reversal = new LegReversal(revShoulder, revHinge, revKnee);
        trajectory = new ArrayList();
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

    /*
    INVERSE AND FORWARDS KINEMATICS CONTROLS CODE 
    */

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

    // NOW WE NEED TO GENERATE A TRAJECTORY OF THETAS AND FOLLOW THAT
    public boolean traverseTo(double xD, double yD, double zD, double speedMAX) {
        LegThetas thetas = RobotMath.inverseKinematics(model, xD, yD, zD);
        LegPosition targetPosition = RobotMath.calculateLegPosition(thetas);
        boolean there = executeCMD(targetPosition, speedMAX);
        //need to tell the other methods we've arrived at the point
        return there;
    }

    //SENDS LEG TO HOME POSITION
    public void home(double speedMAX) {
        executeCMD(homeCMD, speedMAX);
    }

    /*
    COMMAND EXECUTION CODE - PID ARBITRATION/SATURATION SYSTEM 
    */
    public boolean executeCMD(LegPosition targetPosition, double speedMAX) {
        double[] CMDS = {targetPosition.shoulder, targetPosition.hinge, targetPosition.knee};

        double epsilon = 0.1;
        boolean shoulderThere = false;
        boolean hingeThere = false;
        boolean kneeThere = false; 

        position = getPosition();

        if (Math.abs(position.shoulder-CMDS[0])>epsilon) {
            double speed;

            if(Math.abs(CMDS[0]-position.shoulder)>speedMAX){
                speed = speedMAX*Math.signum((CMDS[0]-position.shoulder));
            }
            else {
                speed = (CMDS[0]-position.shoulder);
            }

            position.shoulder = position.shoulder+speed;
        }
        else {
            position.shoulder=CMDS[0];
            shoulderThere = true;
        }

        if (Math.abs(position.hinge-CMDS[1])>epsilon) {
            double speed;

            if(Math.abs(CMDS[1]-position.hinge)>speedMAX){
                speed = speedMAX*Math.signum((CMDS[1]-position.hinge));
            }
            else {
                speed = (CMDS[1]-position.hinge);
            }

            position.hinge = position.hinge+speed;
        } 
        else {
            position.hinge = CMDS[1];
            hingeThere = true;
        }

        if (Math.abs(position.knee-CMDS[2])>epsilon) {
            double speed;

            if(Math.abs(CMDS[2]-position.knee)>speedMAX){
                speed = speedMAX*Math.signum((CMDS[2]-position.knee));
            }
            else {
                speed = (CMDS[2]-position.knee);
            }

            position.knee = position.knee+speed;
        }
        else{
            position.knee = CMDS[2];
            kneeThere = true;
        }

        set(position);
        return shoulderThere && hingeThere && kneeThere;
    }

    /*
    CODE INVOVLING TRAJECTORY GENERATION, CLEARING, AND EXECUTION 
    */
    public boolean addPoint(double xD, double yD, double zD) {
        double l1 = model.L3; //Math.sqrt(L1*L1+L3*L3);
        double l2 = model.L6; //Math.sqrt(L4*L4+L6*L6);
        double r = Math.sqrt(xD*xD+yD*yD);

        //checks if the point has a solution in IK
        if (RobotMath.makesValidTriangle(l1, l2, r)) {
            trajectory.add(new double[]{xD, yD, zD});
            return true;
        }
        return false; 
    }

    public void clearTrajectory() {
        trajectory = new ArrayList();
    }

    public void move(double speedMAX){
        //if there's a trajectory we will be following that
        if (trajectory.size()>0) {
            //get the point we want to go to
            double[] desired = trajectory.get(0);
            //go to that point
            boolean there = traverseTo(desired[0], desired[1], desired[2], speedMAX);
            //only if we have reached that point, should we be telling the leg to go to the next point 
            if (there) {
                trajectory.remove(0);
            }
        }
        //otherwise we will be staying still
        else{
            position = getPosition();
            set(position);
        }
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