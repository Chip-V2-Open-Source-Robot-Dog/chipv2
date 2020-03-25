package edu.mit.chip.leg;

public class LegReversal {
    public double shoulderMult, hingeMult, kneeMult;

    public LegReversal(boolean shoulder, boolean hinge, boolean knee) {
        shoulderMult = shoulder ? -1 : 1;
        hingeMult = hinge ? -1 : 1;
        kneeMult = knee ? -1 : 1;
    }
}