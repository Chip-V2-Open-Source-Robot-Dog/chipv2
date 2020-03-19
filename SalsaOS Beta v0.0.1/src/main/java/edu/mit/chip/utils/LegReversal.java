package edu.mit.chip.utils;

public class LegReversal {
    public double shoulderMult, hingeMult, kneeMult;

    public LegReversal(boolean shoulder, boolean hinge, boolean knee) {
        shoulderMult = shoulder ? 1 : 0;
        hingeMult = hinge ? 1 : 0;
        kneeMult = knee ? 1 : 0;
    }
}