package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AllianceColors;
import org.firstinspires.ftc.teamcode.Utilities.GamerMath;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import static java.lang.Math.toDegrees;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;
import static org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions.angleWrap;

public class Sensors {

    private Robot robot;

    // Sensors
    public BNO055IMU imu;

    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    //public DcMotor leftVerticalEncoder;
    //public DcMotor rightVerticalEncoder;


    public DistanceSensor distanceSensor;

    public final double WALL_DETECT_DIST = 13; // Max inches robot can detect wall at using distance sensor
    public final double HOLDING_STONE_DIST = 6; // How many inches away the stone can be for pepeSMASH to goSMASH!

    public ColorSensor lineSensor;
    public TouchSensor allianceColorSelector;


    //public DistanceSensor pulleySensor;


    public void initHardware(HardwareMap aHwMap, Robot theRobot) {

        robot = theRobot;
        imu = aHwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) pause(15);

        //horizontalEncoder = aHwMap.get(DcMotor.class, "horizontalEncoder");
        //verticalEncoder = aHwMap.get(DcMotor.class, "verticalEncoder");
        //leftVerticalEncoder = aHwMap.get(DcMotor.class, "leftVerticalEncoder");
        //rightVerticalEncoder = aHwMap.get(DcMotor.class, "rightVerticalEncoder");
        //resetEncoders();

        lineSensor = aHwMap.get(ColorSensor.class, "lineSensor");
        distanceSensor = aHwMap.get(DistanceSensor.class, "stoneDistanceSensor");
        allianceColorSelector = aHwMap.get(TouchSensor.class, "stoneBumpSensor");
        if (robot.currentOpModeType == OpModeTypes.AUTO) {
            lineSensor.enableLed(true);
        }

    }


    public double getHorizontalEncoder() { return horizontalEncoder.getCurrentPosition(); }


    public double getVerticalEncoder() {

        return verticalEncoder.getCurrentPosition();
        //double leftValue = leftVerticalEncoder.getCurrentPosition();
        //double rightValue = rightVerticalEncoder.getCurrentPosition();
        //return (leftValue + rightValue)/2;

    }


    public void resetEncoders() {

        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getWorldAngleDeg() { return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle); }

    public double getWorldAngleRad() { return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle); }



    /*
    public boolean intookStone() {
        // Color sensor detects if yellow stone is above stone holding cell
        float hue = getColorSensorHSV(stoneSensor)[0];

        boolean isPossessing = false;

        if (hue > 40 && hue < 70) {
            isPossessing = true;
        }

        return isPossessing;
    }*/


    //Gets distance from sensor in back of robot to anything in the front of the robot
    public double getDistance() {
        return distanceSensor.getDistance(INCH);
    }

    // Gets distance from front of the robot to anything it front of it
    public double getDistanceFromFront() {
        return getDistance()-WALL_DETECT_DIST;
    }


    /** If the distance sensor detects a wall at 13 inches (sensor distance from front of robot)
     * This won't work if a stone or anything is blocking the distance sensor
     */
    public boolean frontTouchingWall() {

        double distanceInches = distanceSensor.getDistance(INCH);
        boolean frontTouchingWall = false;

        if (distanceInches < WALL_DETECT_DIST) {
            frontTouchingWall = true;
        }
        return frontTouchingWall;
    }


    // If stone is within distance to be considered inside robot
    public boolean holdingStone() {

        double distanceInches = distanceSensor.getDistance(INCH);
        boolean holdingStone = false;

        if (distanceInches < HOLDING_STONE_DIST) {
            holdingStone = true;
        }

        return holdingStone;
    }


    // IDK how this works but it does
    public boolean isOverLine() {
        return lineSensor.argb() < 50364673;
    }


    public AllianceColors getSelected() {
        AllianceColors allianceColor;
        if (allianceColorSelector.isPressed()) {
            allianceColor = AllianceColors.RED;
        } else {
            allianceColor = AllianceColors.BLUE;
        }

        return allianceColor;
    }

}
