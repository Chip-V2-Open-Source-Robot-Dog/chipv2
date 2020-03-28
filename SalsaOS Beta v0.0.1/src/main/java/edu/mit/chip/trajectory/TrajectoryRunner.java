package edu.mit.chip.trajectory;

import java.util.ArrayList;

import edu.mit.chip.Robot;
import edu.mit.chip.utils.SpeedSet;

public class TrajectoryRunner {
    public enum State {
        LOADING,
        SERVOING,
        PAUSED;
    }

    private Robot robot;
    private SpeedSet speedSet;
    private ArrayList<Waypoint> waypoints;

    private State state;

    private ServoLegCommand frontLeftLegCommand, frontRightLegCommand, backLeftLegCommand, backRightLegCommand;

    public TrajectoryRunner(Robot robot, SpeedSet speedSet, Waypoint... waypoints) {
        this.robot = robot;
        this.speedSet = speedSet;

        this.waypoints = new ArrayList<Waypoint>();
        for (Waypoint wp : waypoints) {
            this.waypoints.add(wp);
        }

        state = State.LOADING;
    }

    public void reset() {
        state = State.LOADING;
        waypoints.clear();
    }
    
    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint wp : waypoints) {
            this.waypoints.add(wp);
        }
    }

    public void pause() {
        state = State.PAUSED;
    }

    public void resume() {
        state = State.LOADING;
    }

    public void tick() {
        if (state == State.LOADING && waypoints.size() > 0) {
            frontLeftLegCommand = new ServoLegCommand(robot.frontLeftLeg, waypoints.get(0).frontLeft, speedSet.frontLeft);
            frontRightLegCommand = new ServoLegCommand(robot.frontRightLeg, waypoints.get(0).frontRight, speedSet.frontRight);
            backLeftLegCommand = new ServoLegCommand(robot.backLeftLeg, waypoints.get(0).backLeft, speedSet.backLeft);
            backRightLegCommand = new ServoLegCommand(robot.backRightLeg, waypoints.get(0).backRight, speedSet.backRight);

            state = State.SERVOING;
        }
        else if (state == State.SERVOING) {
            frontLeftLegCommand.tick();
            frontRightLegCommand.tick();
            backLeftLegCommand.tick();
            backRightLegCommand.tick();

            if (frontLeftLegCommand.isDone() && frontRightLegCommand.isDone() && backLeftLegCommand.isDone() && backRightLegCommand.isDone()) {
                waypoints.remove(0);
                state = State.LOADING;
            }
        }
        else {
            // Do nothing and wait for another command to be added...
        }
    }
}