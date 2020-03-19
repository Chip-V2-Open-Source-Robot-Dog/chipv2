package edu.mit.chip.utils;

public class RobotMath {
    public static double calculateEncoderTicks(double theta) {
        return theta * 100.0/(2*Math.PI);
    }

    public static double calculateTheta(double encoderPosition) {
        return encoderPosition * (2*Math.PI)/100.0;
    }
}