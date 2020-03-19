package edu.mit.chip.utils;

public class RobotMath {
    public static LegPosition calculateLegPosition(double shoulderTheta, double hingeTheta, double kneeTheta) {
        double shoulderPos = RobotMath.calculateEncoderTicks(shoulderTheta);
        double hingePos = RobotMath.calculateEncoderTicks(hingeTheta);
        double kneePos = RobotMath.calculateEncoderTicks(kneeTheta);

        return new LegPosition(shoulderPos, hingePos, kneePos);
    }

    public static double calculateEncoderTicks(double theta) {
        return theta * 100.0/(2*Math.PI);
    }

    public static double calculateTheta(double encoderPosition) {
        return encoderPosition * (2*Math.PI)/100.0;
    }

    public static boolean makesValidTriangle(double a, double b, double c) { 
        return (a + b <= c || a + c <= b || b + c <= a);
    } 
}