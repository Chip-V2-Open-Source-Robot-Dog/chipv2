package edu.mit.chip.utils;

public class LegModel {
    public double L1;
    public double L2;
    public double L3;
    public double L4;
    public double L5;
    public double L6;
    
    public LegModel(double[] model) {        
        if (model.length == 6) {
            L1 = model[0];
            L2 = model[1];
            L3 = model[2];
            L4 = model[3];
            L5 = model[4];
            L6 = model[5];
        }
        else {
            throw new IllegalArgumentException("Leg model does not match internal model. Expected 6 arguments, got " + model.length + ".");
        }
    }

    public LegModel(double L1, double L2, double L3, double L4, double L5, double L6) {
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
        this.L4 = L4;
        this.L5 = L5;
        this.L6 = L6;
    }
}