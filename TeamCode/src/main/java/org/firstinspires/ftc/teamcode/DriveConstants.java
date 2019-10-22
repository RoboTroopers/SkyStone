package org.firstinspires.ftc.teamcode;

public class DriveConstants {

    /*
     *
     * All the robot's constants relating to its motion and measurements
     *
     */

    public static final double WHEEL_RADIUS = 2;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS*2*Math.PI;

    // length between front & back wheels
    public static final double WHEEL_BASE = 10;

    // width between left and right wheels
    public static final double TRACK_WIDTH = 16;

    /*
     * diameter of the circumcircle of the robot, really the distance between centers of 2 diagonal
     * wheels (hypotenuse of wheelbase and track width)
     */
    public static final double CIRCUMDIAMETER = Math.hypot(WHEEL_BASE, TRACK_WIDTH);
    //Circumference of circle
    public static final double CIRCUMCIRCLE_CIRCUMFERENCE = CIRCUMDIAMETER*Math.PI;


    public static double inchesToTicks(int inches) {
        return (inches / WHEEL_CIRCUMFERENCE) * 360;

    }

    public static double ticksToInches(int inches) {
        return (inches * WHEEL_CIRCUMFERENCE) / 360;

    }




}
