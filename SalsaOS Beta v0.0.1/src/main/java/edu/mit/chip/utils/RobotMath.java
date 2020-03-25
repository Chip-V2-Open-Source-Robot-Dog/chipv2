package edu.mit.chip.utils;

public class RobotMath {
    public static LegThetas inverseKinematics(LegModel legModel, double xD, double yD, double zD) {
        double l1 = legModel.L3; //Math.sqrt(L1*L1+L3*L3);
        double l2 = legModel.L6; //Math.sqrt(L4*L4+L6*L6);

        double rSquared = xD*xD+yD*yD;
        double phi = Math.atan2(yD, xD);
        double kneeTheta = Math.acos((rSquared-l1*l1-l2*l2)/(-2.0*l1*l2));
        double alpha = Math.acos((l2*l2-l1*l1-rSquared)/(-2.0*l1*Math.sqrt(rSquared)));
        double shoulderTheta = phi-alpha;
        double hingeTheta = Math.asin(zD/yD);

        return new LegThetas(shoulderTheta, hingeTheta, kneeTheta);
    }
    
    public static LegPosition calculateLegPosition(LegThetas thetas) {
        return RobotMath.calculateLegPosition(thetas.shoulder, thetas.hinge, thetas.knee);
    }

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