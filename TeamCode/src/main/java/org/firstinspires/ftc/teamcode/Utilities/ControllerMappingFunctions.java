package org.firstinspires.ftc.teamcode.Utilities;

public class ControllerMappingFunctions {
    
    public double getJoystickAngleRad(double xInput, double yInput) {
        
        double radians = Math.tan(yInput/xInput);
        return radians;
        
    }

    public double getJoystickAngleDeg(double xInput, double yInput) {

        double degrees = Math.toDegrees(Math.tan(yInput/xInput));
        return degrees;

    }

    public double getJoystickDistanceFromCenter(double xInput, double yInput) {

        double distance = Math.hypot(xInput, yInput);
        return distance;

    }
    
    public double convertExponentialSensitivity(double joystickValue) {
        double convertedValue = Math.pow(joystickValue, 3);
        return convertedValue;
        
    }
    
}
