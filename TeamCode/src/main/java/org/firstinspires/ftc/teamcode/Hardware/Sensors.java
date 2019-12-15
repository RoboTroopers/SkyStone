package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;
import org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions;

public class Sensors {

    private Robot robot;

    // Sensors
    public BNO055IMU imu;

    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    //public DcMotor leftVerticalEncoder;
    //public DcMotor rightVerticalEncoder;


    public DistanceSensor stoneDistanceSensor;

    public final double WALL_DETECT_DIST = 13; // Max inches robot can detect wall at using distance sensor
    public final double HOLDING_STONE_DIST = 6; // How many inches away the stone can be for pepeSMASH to goSMASH!

    public ColorSensor lineSensor;
    public TouchSensor stoneBumpSensor;


    //public DistanceSensor pulleySensor;


    public void initHardware(HardwareMap aHwMap, Robot theRobot) {

        robot = theRobot;
        imu = aHwMap.get(BNO055IMU.class, "imu");

        //horizontalEncoder = aHwMap.get(DcMotor.class, "horizontalEncoder");
        //verticalEncoder = aHwMap.get(DcMotor.class, "verticalEncoder");
        //leftVerticalEncoder = aHwMap.get(DcMotor.class, "leftVerticalEncoder");
        //rightVerticalEncoder = aHwMap.get(DcMotor.class, "rightVerticalEncoder");
        //resetEncoders();

        lineSensor = aHwMap.get(ColorSensor.class, "lineSensor");
        stoneDistanceSensor = aHwMap.get(DistanceSensor.class, "stoneDistanceSensor");
        stoneBumpSensor = aHwMap.get(TouchSensor.class, "stoneBumpSensor");
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


    public double getWorldAngleRad() { return MathFunctions.angleWrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle); }



    public float[] getColorSensorHSV(ColorSensor thisColorSensor) {

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // convert the RGB values to HSV values.
        Color.RGBToHSV((int) (thisColorSensor.red() * 255),
                (int) (thisColorSensor.green() * 255),
                (int) (thisColorSensor.blue() * 255),
                hsvValues);

        return hsvValues;

    }


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


    // If the distance sensor detects a wall at 13 inches (sensor distance from front of robot)
    public boolean frontTouchingWall() {

        double distanceInches = stoneDistanceSensor.getDistance(DistanceUnit.INCH);
        boolean frontToucingWall = false;

        if (distanceInches < WALL_DETECT_DIST) {
            frontToucingWall = true;
        }
        return frontToucingWall;
    }


    //
    public boolean holdingStone() {

        double distanceInches = stoneDistanceSensor.getDistance(DistanceUnit.INCH);
        boolean holdingStone = false;

        if (distanceInches < HOLDING_STONE_DIST) {
            holdingStone = true;
        }

        return holdingStone;
    }


    public boolean stoneAtBack() {
        return stoneBumpSensor.isPressed();
    }



    public boolean overLine() {

        float hue = getColorSensorHSV(lineSensor)[0];
        float saturation = getColorSensorHSV(lineSensor)[1];
        boolean isOvertape = false;

        if (((hue >= 0 && hue <= 25) || (hue >= 180 && hue <= 250)) && saturation > 0.4) {
            isOvertape = true;
        }

        return isOvertape;

    }



}
