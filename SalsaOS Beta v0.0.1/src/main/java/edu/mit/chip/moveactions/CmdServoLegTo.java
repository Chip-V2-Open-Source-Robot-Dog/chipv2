package edu.mit.chip.moveactions;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.utils.FootPosition;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.RobotMath;
import edu.wpi.first.wpilibj.command.Command;

public class CmdServoLegTo extends Command {
    private final double MAX_ERROR = 0.1;

    private Leg leg;
    private double maxSpeed;

    private FootPosition targetFootPosition;
    private LegPosition targetLegPosition;

    private LegPosition legPosition;
    private double error;

    private boolean shoulderAtTarget = false;
    private boolean hingeAtTarget = false;
    private boolean kneeAtTarget = false;

    public CmdServoLegTo(Leg leg, FootPosition targetPosition, double maxSpeed) {
        this.leg = leg;
        this.targetFootPosition = targetPosition;
        this.maxSpeed = maxSpeed;
    }

    @Override
    protected void initialize() {
        final double[] thetas = leg.inverseKinematics(targetFootPosition.x, targetFootPosition.y, targetFootPosition.z);
        targetLegPosition = RobotMath.calculateLegPosition(thetas[0], thetas[1], thetas[2]);
    }

    @Override
    protected void execute() {
        legPosition = leg.getPosition();

        error = targetLegPosition.shoulder - legPosition.shoulder;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.shoulder += RobotMath.clip(error, -maxSpeed, maxSpeed);
            shoulderAtTarget = false;
        }
        else {
            legPosition.shoulder = targetLegPosition.shoulder;
            shoulderAtTarget = true;
        }

        error = targetLegPosition.hinge - legPosition.hinge;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.hinge += RobotMath.clip(error, -maxSpeed, maxSpeed);
            hingeAtTarget = false;
        }
        else {
            legPosition.hinge = targetLegPosition.hinge;
            hingeAtTarget = true;
        }

        error = targetLegPosition.knee - legPosition.knee;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.knee += RobotMath.clip(error, -maxSpeed, maxSpeed);
            kneeAtTarget = false;
        }
        else {
            legPosition.knee = targetLegPosition.knee;
            kneeAtTarget = true;
        }

        leg.set(legPosition);
    }

    @Override
    protected boolean isFinished() {
        return shoulderAtTarget && hingeAtTarget && kneeAtTarget;
    }

    @Override
    protected void end() {
        
    }
}