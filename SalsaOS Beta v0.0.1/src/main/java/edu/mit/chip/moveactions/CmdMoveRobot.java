package edu.mit.chip.moveactions;

import edu.mit.chip.Robot;
import edu.mit.chip.utils.FootPosition;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdMoveRobot extends CommandGroup {
    public CmdMoveRobot(Robot robot,
            FootPosition frontLeft,  double frontLeftMaxSpeed,
            FootPosition frontRight, double frontRightMaxSpeed,
            FootPosition backLeft,   double backLeftMaxSpeed,
            FootPosition backRight,  double backRightMaxSpeed
        ) {
        addParallel(new CmdServoLegTo(robot.frontLeftLeg, frontLeft, frontLeftMaxSpeed));
        addParallel(new CmdServoLegTo(robot.frontRightLeg, frontRight, frontRightMaxSpeed));
        addParallel(new CmdServoLegTo(robot.backLeftLeg, backLeft, backLeftMaxSpeed));
        addParallel(new CmdServoLegTo(robot.backRightLeg, backRight, backRightMaxSpeed));
    }
}