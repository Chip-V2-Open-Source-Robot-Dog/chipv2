package edu.mit.chip.setupactions;

import edu.mit.chip.mechanisms.Leg;
import edu.mit.chip.utils.LegPosition;
import edu.wpi.first.wpilibj.command.Command;

public class ZeroLegAction extends SetupAction {
    public ZeroLegAction(Leg leg, String legName) {
        super("Zero " + legName + " Leg");

        this.cmd = new Command() {
			@Override
			protected boolean isFinished() {
                leg.setEncoders(new LegPosition(0, 0, 0));
				return false;
			}
        };
    }
}