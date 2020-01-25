package org.firstinspires.ftc.teamcode.Utilities;

public class GamerMath {


    // Cast rounded value to int because idk why it doesn't do this by default.
    public static int castRound(double number) { return (int)Math.round(number); }



    /** Ensure input is inside a certain range. */
    public static double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

    public static int clamp(int val, int min, int max) { return Math.max(min, Math.min(max, val)); }


    /** Ensure input is within range, but if the number is negative flip the signs
     * (prevents forcing absolute value to be used).
     */
    public static double clampSigned(double val, double min, double max) {
        double number;
        if (val >= 0.0)
            number = clamp(val, min, max);
        else number = clamp(-val, -max, -min);
        return number;
    }

    public static int clampSigned(int val, int min, int max) {
        int number;
        if (val >= 0)
            number = clamp(val, min, max);
        else number = clamp(-val, -max, -min);
        return number;
    }


    public static double angleWrapDeg(double angle) {
        if (angle <= -180) angle += 360;
        if (angle >= 180) angle -= 360;
        return angle;

    }



}
