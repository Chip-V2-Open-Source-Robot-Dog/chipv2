package edu.mit.chip.setpoint;

import edu.mit.chip.Robot;
import edu.mit.chip.leg.LegType;
import edu.mit.chip.utils.SpeedSet;

public class SetpointManager {
    private enum State {
        HOLDING,
        PAUSED;
    }

    private State state;
    private HoldSetpointCommand frontLeftLegCommand, frontRightLegCommand, backLeftLegCommand, backRightLegCommand;

    public SetpointManager(Robot robot, SpeedSet speedSet) {
        frontLeftLegCommand = new HoldSetpointCommand(robot.frontLeftLeg, speedSet.frontLeft);
        frontRightLegCommand = new HoldSetpointCommand(robot.frontRightLeg, speedSet.frontRight);
        backLeftLegCommand = new HoldSetpointCommand(robot.backLeftLeg, speedSet.backLeft);
        backRightLegCommand = new HoldSetpointCommand(robot.backRightLeg, speedSet.backRight);

        state = State.PAUSED;
    }

    public void pause() {
        state = State.PAUSED;
    }

    public void resume() {
        state = State.HOLDING;
    }

    public void updateSetpoint(LegType legType, double x, double y, double z) {
        HoldSetpointCommand cmd;
        switch (legType) {
            case FRONT_LEFT:
                cmd = frontLeftLegCommand;
                break;
            case FRONT_RIGHT:
                cmd = frontRightLegCommand;
                break;
            case BACK_LEFT:
                cmd = backLeftLegCommand;
                break;
            case BACK_RIGHT:
                cmd = backRightLegCommand;
                break;
            default:
                cmd = backRightLegCommand;
                break;
        }

        cmd.setTarget(x, y, z);
    }

    public void tick() {
        if (state == State.HOLDING) {
            frontLeftLegCommand.tick();
            frontRightLegCommand.tick();
            backLeftLegCommand.tick();
            backRightLegCommand.tick();
        }
    }
}