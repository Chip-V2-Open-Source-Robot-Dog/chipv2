package edu.mit.chip.trajectory;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.utils.FootPosition;
import edu.mit.chip.utils.LegPosition;
import edu.mit.chip.utils.RobotMath;

public class ServoLegCommand {
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

    private TickState state;

    public ServoLegCommand(Leg leg, FootPosition targetPosition, double maxSpeed) {
        this.leg = leg;
        this.targetFootPosition = targetPosition;
        this.maxSpeed = maxSpeed;

        state = TickState.INITIALIZING;
    }

    private enum TickState {
        INITIALIZING,
        EXECUTING,
        ENDING,
        DONE;
    }

    public void tick() {
        switch (state) {
            case INITIALIZING:
                initialize();
                state = TickState.EXECUTING;
                break;
            case EXECUTING:
                execute();
                if (isFinished())
                    state = TickState.ENDING;
                break;
            case ENDING:
                end();
                state = TickState.DONE;
                break;
            case DONE:
                break;
        }
    }

    public boolean isDone() {
        return state == TickState.DONE;
    }

    private void initialize() {
        final double[] thetas = leg.inverseKinematics(targetFootPosition.x, targetFootPosition.y, targetFootPosition.z);
        targetLegPosition = RobotMath.calculateLegPosition(thetas[0], thetas[1], thetas[2]);
    }

    private void execute() {
        legPosition = leg.getPosition();

        error = targetLegPosition.shoulder - legPosition.shoulder;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.shoulder += RobotMath.clip(error, -maxSpeed, maxSpeed);
            shoulderAtTarget = false;
        } else {
            legPosition.shoulder = targetLegPosition.shoulder;
            shoulderAtTarget = true;
        }

        error = targetLegPosition.hinge - legPosition.hinge;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.hinge += RobotMath.clip(error, -maxSpeed, maxSpeed);
            hingeAtTarget = false;
        } else {
            legPosition.hinge = targetLegPosition.hinge;
            hingeAtTarget = true;
        }

        error = targetLegPosition.knee - legPosition.knee;
        if (Math.abs(error) > MAX_ERROR) {
            legPosition.knee += RobotMath.clip(error, -maxSpeed, maxSpeed);
            kneeAtTarget = false;
        } else {
            legPosition.knee = targetLegPosition.knee;
            kneeAtTarget = true;
        }

        leg.set(legPosition);
    }

    private boolean isFinished() {
        return shoulderAtTarget && hingeAtTarget && kneeAtTarget;
    }

    private void end() {
        leg.set(targetLegPosition);
    }
}