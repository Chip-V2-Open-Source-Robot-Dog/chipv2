package edu.mit.chip.utils;

public class RobotMath {
    public static LegPosition calculateLegPosition(double shoulderTheta, double hingeTheta, double kneeTheta) {
        double shoulderPos = RobotMath.calculateEncoderRevolutions(shoulderTheta);
        double hingePos = RobotMath.calculateEncoderRevolutions(hingeTheta);
        double kneePos = RobotMath.calculateEncoderRevolutions(kneeTheta);

        return new LegPosition(shoulderPos, hingePos, kneePos);
    }

    public static double calculateEncoderRevolutions(double theta) {
        return theta * 100.0/(2*Math.PI);
    }

    public static double calculateTheta(double encoderPosition) {
        return encoderPosition * (2*Math.PI)/100.0;
    }

    public static boolean makesValidTriangle(double a, double b, double c) { 
        return !(a + b <= c || a + c <= b || b + c <= a);
    } 

    public static double clip(double value, double min, double max) {
        if (value >= max) return max;
        if (value <= min) return min;
        return value;
    }
}