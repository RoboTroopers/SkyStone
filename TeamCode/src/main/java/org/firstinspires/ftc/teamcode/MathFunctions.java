package org.firstinspires.ftc.teamcode;


import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.DriveConstants.WHEEL_CIRCUMFERENCE;


public class MathFunctions {
    
    //Keeps angle within -180 to 180 degrees while preserving angle measure
    public static double angleWrap(double angle) {
        while (angle < -PI)
            angle += 2*PI;
        
        while (angle > PI)
            angle -= 2*PI;
        
        return angle;
        
    }
    
    
    public static double inchesToTicks(double inches) {
        double ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
        return ticks;
        
    }
    
    
    public static double ticksToInches(double ticks) {
        double inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
        return inches;
        
    }
    
    
    public static void pause(int sleepTime) {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    
}
