package org.firstinspires.ftc.teamcode;


import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.DriveConstants.*;


public class MathFunctions {
    
    /**
     *
     * Keeps angle within -180 to 180 degrees while preserving angle measure
     *
     */
    
    public static double angleWrap(double angle) {
        while (angle < -PI)
            angle += 2*PI;
        
        while (angle > PI)
            angle -= 2*PI;
        
        return angle;
        
    }
    
    
    public static double restrictToRange(double value, double bottom, double top) {
        if (value < bottom) value = bottom;
        if (value > top) value = top;
        return value;
    }


    public static double inchesToTicks(double inches) {
        double ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
        return ticks;

    }


    public static double ticksToInches(double ticks) {
        double inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
        return inches;

    }
    
    
}
