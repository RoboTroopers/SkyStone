package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class Sensors extends HardwareComponent {

    // Sensors
    public BNO055IMU imu;

    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    //public DcMotor leftVerticalEncoder;
    //public DcMotor rightVerticalEncoder;


    public DistanceSensor stoneDistanceSensor;

    public final double HOLDING_STONE_DIST = 3; // How many inches away the stone can be for pepeSMASH to goSMASH!
    public final double INTAKING_DIST = 10; // Farthest distance stone can be from distance sensor


    public ColorSensor lineSensor;
    public TouchSensor allianceColorSelector;

    //public DistanceSensor pulleySensor;



    public Sensors(Robot theRobot, OpMode opMode) {
        super(theRobot, opMode);
    }



    public void init(HardwareMap aHwMap) {

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
        stoneDistanceSensor = aHwMap.get(DistanceSensor.class, "stoneDistanceSensor");
        allianceColorSelector = aHwMap.get(TouchSensor.class, "touchSensor");
        lineSensor.enableLed(true);

    }

    @Deprecated
    public double getHorizontalEncoder() { return horizontalEncoder.getCurrentPosition(); }

    @Deprecated
    public double getVerticalEncoder() {

        return verticalEncoder.getCurrentPosition();
        //double leftValue = leftVerticalEncoder.getCurrentPosition();
        //double rightValue = rightVerticalEncoder.getCurrentPosition();
        //return (leftValue + rightValue)/2;
    }

    @Deprecated
    public void resetEncoders() {

        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getWorldAngleDeg() { return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle); }

    public double getWorldAngleRad() { return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle); }


    @Deprecated
    //Gets distance from sensor in back of robot to anything in the front of the robot
    public double getStoneDistance() {
        return stoneDistanceSensor.getDistance(INCH);
    }


    @Deprecated
    // If stone is within distance to be considered inside robot
    public boolean holdingStone() {

        double distanceInches = getStoneDistance();
        boolean holdingStone = false;

        if (distanceInches < HOLDING_STONE_DIST) {
            holdingStone = true;
        }

        return holdingStone;
    }


    @Deprecated
    // If stone is at the distance considered to be inside the intake
    public boolean intakingStone() {

        double distanceInches = getStoneDistance();
        boolean holdingStone = false;

        if (distanceInches < INTAKING_DIST && distanceInches > HOLDING_STONE_DIST) {
            holdingStone = true;
        }

        return holdingStone;
    }


    // IDK how this works but it does
    public boolean isOverLine() {
        return lineSensor.argb() < 50364673;
    }


    public boolean isRedSelected() {
        boolean isRed = false;
        if (allianceColorSelector.isPressed()) {
            isRed = true;
        }

        return isRed;
    }

}
