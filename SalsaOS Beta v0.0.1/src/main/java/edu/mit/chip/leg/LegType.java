package edu.mit.chip.leg;

public enum LegType {
    FRONT_LEFT("fl"),
    FRONT_RIGHT("fr"),
    BACK_LEFT("bl"),
    BACK_RIGHT("br");
    
    public String prefix;
    private LegType(String prefix) {
        this.prefix = prefix;
    }
}