package edu.mit.chip.leg;

public enum LegType {
    FRONT_LEFT("fL"),
    FRONT_RIGHT("fR"),
    BACK_LEFT("bL"),
    BACK_RIGHT("bR");
    
    public String prefix;
    private LegType(String prefix) {
        this.prefix = prefix;
    }

    public String key(String subscript) {
        return prefix + "_" + subscript;
    }
}