package edu.mit.chip.setupactions;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetupActionChooser {
    private SendableChooser<Command> sendableChooser;

    public SetupActionChooser(SetupAction... actions) {
        sendableChooser = new SendableChooser<Command>();

        NoneAction noneAction = new NoneAction();
        sendableChooser.setDefaultOption(noneAction.name, noneAction.cmd);
        for (SetupAction action : actions) {
            sendableChooser.addOption(action.name, action.cmd);
        }
    }

    public Command getChosenActionCmd() {
        return sendableChooser.getSelected();
    }

    public void putOnDashboard() {
        SmartDashboard.putData("Setup Actions", sendableChooser);
    }
}