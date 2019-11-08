package org.firstinspires.ftc.teamcode.Utilities;

public class JoystickFunctions {
    
    public static double getJoystickAngleRad(double xInput, double yInput) {
        
        double radians = Math.tan(yInput/xInput);
        return radians;
        
    }

    public static double getJoystickAngleDeg(double xInput, double yInput) {

        double degrees = Math.toDegrees(Math.tan(yInput/xInput));
        return degrees;

    }

    public static double getJoystickDistanceFromCenter(double xInput, double yInput) {

        double distance = Math.hypot(xInput, yInput);
        return distance;

    }
    
    public static double convertExponentialSensitivity(double joystickValue) {
        double convertedValue = Math.pow(joystickValue, 3);
        return convertedValue;
        
    }
    
    
}
