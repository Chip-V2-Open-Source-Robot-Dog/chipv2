package edu.mit.chip.leg;

public enum LegType {
    FRONT_LEFT("Front Left", "fL"),
    FRONT_RIGHT("Front Right", "fR"),
    BACK_LEFT("Back Left", "bL"),
    BACK_RIGHT("Back Right", "bR");
    
    public String name, prefix;
    private LegType(String name, String prefix) {
        this.name = name;
        this.prefix = prefix;
    }

    public String key(String subscript) {
        return prefix + "_" + subscript;
    }
}