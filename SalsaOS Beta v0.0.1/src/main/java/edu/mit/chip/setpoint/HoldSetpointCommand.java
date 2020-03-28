package edu.mit.chip.setpoint;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.leg.FootPosition;
import edu.mit.chip.leg.LegPosition;
import edu.mit.chip.leg.LegThetas;
import edu.mit.chip.utils.ChipCommand;
import edu.mit.chip.utils.RobotMath;

public class HoldSetpointCommand extends ChipCommand{
    private final double MAX_ERROR = 0.1;

    private Leg leg;
    private double maxSpeed;

    private LegPosition targetLegPosition;

    private LegPosition legPosition;
    private double error;

    private boolean shoulderAtTarget = false;
    private boolean hingeAtTarget = false;
    private boolean kneeAtTarget = false;

    public HoldSetpointCommand(Leg leg, double maxSpeed) {
        super();

        this.leg = leg;
        this.maxSpeed = maxSpeed;

        final FootPosition currentPosition = leg.getFootPosition();
        setTarget(currentPosition.x, currentPosition.y, currentPosition.z);
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public void setTarget(double x, double y, double z) {
        final LegThetas thetas = RobotMath.inverseKinematics(leg.model, x, y, z);
        targetLegPosition = RobotMath.calculateLegPosition(thetas);
    } 

    @Override
    protected void initialize() {
        
    }

    @Override
    protected void execute() {
        legPosition = leg.getPosition();

        error = targetLegPosition.shoulder - legPosition.shoulder;
        shoulderAtTarget = Math.abs(error) <= MAX_ERROR;
        if (!shoulderAtTarget) {
            legPosition.shoulder += RobotMath.clip(error, -maxSpeed, maxSpeed);
        } else {
            legPosition.shoulder = targetLegPosition.shoulder;
        }

        error = targetLegPosition.hinge - legPosition.hinge;
        hingeAtTarget = Math.abs(error) <= MAX_ERROR;
        if (!hingeAtTarget) {
            legPosition.hinge += RobotMath.clip(error, -maxSpeed, maxSpeed);
        } else {
            legPosition.hinge = targetLegPosition.hinge;
        }

        error = targetLegPosition.knee - legPosition.knee;
        kneeAtTarget = Math.abs(error) <= MAX_ERROR;
        if (!kneeAtTarget) {
            legPosition.knee += RobotMath.clip(error, -maxSpeed, maxSpeed);
        } else {
            legPosition.knee = targetLegPosition.knee;
        }

        leg.set(legPosition);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        leg.set(targetLegPosition);
    }
}