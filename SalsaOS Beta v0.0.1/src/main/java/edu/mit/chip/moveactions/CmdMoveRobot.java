package edu.mit.chip.moveactions;

import edu.mit.chip.Robot;
import edu.mit.chip.utils.FootPosition;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class CmdMoveRobot extends ParallelCommandGroup {
    public CmdMoveRobot(Robot robot,
            FootPosition frontLeft,  double frontLeftMaxSpeed,
            FootPosition frontRight, double frontRightMaxSpeed,
            FootPosition backLeft,   double backLeftMaxSpeed,
            FootPosition backRight,  double backRightMaxSpeed
        ) {
            super(
                new CmdServoLegTo(robot.frontLeftLeg, frontLeft, frontLeftMaxSpeed),
                new CmdServoLegTo(robot.frontRightLeg, frontRight, frontRightMaxSpeed),
                new CmdServoLegTo(robot.backLeftLeg, backLeft, backLeftMaxSpeed),
                new CmdServoLegTo(robot.backRightLeg, backRight, backRightMaxSpeed)
            );
    }
}