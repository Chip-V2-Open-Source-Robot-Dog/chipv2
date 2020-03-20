package edu.mit.chip.setupactions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class NoneAction extends SetupAction {
    public NoneAction() {
        super("None");

        this.cmd = new CommandBase() {};
    }
}