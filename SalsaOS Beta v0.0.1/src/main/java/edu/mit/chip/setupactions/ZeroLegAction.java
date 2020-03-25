package edu.mit.chip.setupactions;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.leg.LegPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroLegAction extends SetupAction {
    public ZeroLegAction(Leg leg, String legName) {
        super("Zero " + legName + " Leg");

        this.cmd = new CommandBase() {
			@Override
			public void execute() {
                leg.setEncoders(new LegPosition(0, 0, 0));
			}
        };
    }
}