package edu.mit.chip.setupactions;

import edu.wpi.first.wpilibj2.command.Command;

public class SetupAction {
    public String name;
    public Command cmd;

    public SetupAction(String name) {
        this.name = name;
    }
}
