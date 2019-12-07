package org.firstinspires.ftc.teamcode.Globals;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DriveConstants {

    /**
     *
     * All the robot's constants relating to its motion and measurements
     * Every linear measurement uses inches.
     *
     */


    public static final double WHEEL_DIAMETER = 4;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;

    // distance between left and right wheels (width)
    public static final double TRACK_WIDTH = 16;

    // distance between front & back wheels (length)
    public static final double WHEEL_BASE = 10;


    /*
     * CIRCUMCIRCLE: Circle circumscribed about points of a polygon.
     *
     * CIRCUMDIAMETER: Diameter of the circumcircle of the robot, really the distance between ground contact points
     * of 2 diagonal wheels (hypotenuse of wheelbase and track width)
     *
     */

    public static final double CIRCUMDIAMETER = Math.hypot(WHEEL_BASE, TRACK_WIDTH);
    public static final double CIRCUMCIRCLE_CIRCUMFERENCE = CIRCUMDIAMETER*Math.PI;


    //public static final double ENCODER_TICKS_PER_REV = 103;
    //private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(DcMotor.class);
    //private static final double ENCODER_TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    private static final double ENCODER_TICKS_PER_REV = 383.6;


    public static double inchesToTicks(double inches) {
        double ticks = (inches / WHEEL_CIRCUMFERENCE) * ENCODER_TICKS_PER_REV;
        return ticks;

    }

    public static double ticksToInches(double ticks) {
        double inches = (ticks * WHEEL_CIRCUMFERENCE) / ENCODER_TICKS_PER_REV;
        return inches;

    }



}