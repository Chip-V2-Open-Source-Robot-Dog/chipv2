package edu.mit.chip.setupactions;

import edu.wpi.first.wpilibj.command.Command;

public class NoneAction extends SetupAction {
    public NoneAction() {
        super("None");

        this.cmd = new Command() {
			@Override
			protected boolean isFinished() {
				return false;
			}
        };
    }
}