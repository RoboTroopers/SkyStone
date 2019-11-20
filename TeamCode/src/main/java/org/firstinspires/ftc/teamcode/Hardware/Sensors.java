package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions;

public class Sensors {


    // Sensors
    public BNO055IMU imu;

    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    //public DcMotor leftVerticalEncoder;
    //public DcMotor rightVerticalEncoder;


    public ColorSensor colorSensor;
    //public DistanceSensor distanceSensor;


    private boolean possessingStoneLast = false;



    public void initHardware(HardwareMap aHwMap) {

        imu = aHwMap.get(BNO055IMU.class, "imu");
        
        //horizontalEncoder = aHwMap.get(DcMotor.class, "horizontalEncoder");
        //verticalEncoder = aHwMap.get(DcMotor.class, "verticalEncoder");
        //leftVerticalEncoder = aHwMap.get(DcMotor.class, "leftVerticalEncoder");
        //rightVerticalEncoder = aHwMap.get(DcMotor.class, "rightVerticalEncoder");
        //resetEncoders();
        colorSensor = aHwMap.get(ColorSensor.class, "colorSensor");
        //distanceSensor = aHwMap.get(DistanceSensor.class, "distancesSensor");


    }



    public double getHorizontalEncoder() {

        return horizontalEncoder.getCurrentPosition();
    }


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

    

    public double getWorldAngleRad() {
        return MathFunctions.angleWrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle);
    }
    


    public boolean possessingStone() {
        // Color sensor detects if yellow stone is above stone holding cell

        double rgDiff = Math.abs(colorSensor.red() - colorSensor.green());
        double rbDiff = colorSensor.red() - colorSensor.blue();
        double gbDiff = colorSensor.green() - colorSensor.blue();

        boolean possessingStone = false;

        if (rgDiff < rbDiff && rgDiff < gbDiff) {
            // If difference between red and green is less than diff between red and blue and between green and blue

            possessingStone = true;
        }
        possessingStoneLast = possessingStone;
        return possessingStone;

    }
    


}
