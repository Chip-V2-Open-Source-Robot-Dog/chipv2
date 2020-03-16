package edu.mit.chip.setupactions;

import edu.wpi.first.wpilibj.command.Command;

public class SetupAction {
    public String name;
    public Command cmd;

    public SetupAction(String name) {
        this.name = name;
    }
}
