package edu.mit.chip.trajectory;

import java.util.Arrays;
import java.util.List;

import edu.mit.chip.Robot;

public class Trajectory {
    public enum TickState {
        INITIALIZE,
        EXECUTE;
    }

    private Robot robot;
    private SpeedSet speedSet;
    private List<Waypoint> waypoints;

    private TickState state;

    private ServoLegCommand frontLeftLegCommand, frontRightLegCommand, backLeftLegCommand, backRightLegCommand;

    public Trajectory(Robot robot, SpeedSet speedSet, Waypoint... waypoints) {
        this.waypoints = Arrays.asList(waypoints);

        this.state = TickState.INITIALIZE;
    }
    
    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void tick() {
        if (state == TickState.INITIALIZE && waypoints.size() > 0) {
            frontLeftLegCommand = new ServoLegCommand(robot.frontLeftLeg, waypoints.get(0).frontLeft, speedSet.frontLeft);
            frontRightLegCommand = new ServoLegCommand(robot.frontRightLeg, waypoints.get(0).frontRight, speedSet.frontRight);
            backLeftLegCommand = new ServoLegCommand(robot.backLeftLeg, waypoints.get(0).backLeft, speedSet.backLeft);
            backRightLegCommand = new ServoLegCommand(robot.backRightLeg, waypoints.get(0).backRight, speedSet.backRight);

            state = TickState.EXECUTE;
        }
        else if (state == TickState.EXECUTE) {
            frontLeftLegCommand.tick();
            frontRightLegCommand.tick();
            backLeftLegCommand.tick();
            backRightLegCommand.tick();

            if (frontLeftLegCommand.isDone() && frontRightLegCommand.isDone() && backLeftLegCommand.isDone() && backRightLegCommand.isDone()) {
                state = TickState.INITIALIZE;
            }
        }
        else {
            // Do nothing and wait for another command to be added...
        }
    }
}