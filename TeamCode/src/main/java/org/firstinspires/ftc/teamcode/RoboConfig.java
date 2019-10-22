package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.teamcode.DriveConstants.*;



public class RoboConfig {
    
    // Motors and sensors
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;
    public Servo pinger = null;
    
    // Sensors
    public BNO055IMU imu = null;
    private AnalogSensor xEncoder = null;
    private AnalogSensor yEncoder = null;
    
    
    //public static double xSpeed = 0;
    //public static double ySpeed = 0;
    //public static double turnSpeed = 0;
    
    
    // Odometry variables
    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;
    
    
    
    public double getXPos(){
        return worldXPosition;
    }
    
    public double getYPos(){
        return worldYPosition;
    }
    
    
    
    public void initHardware (HardwareMap aHwMap) {
        rightFront = aHwMap.get(DcMotor.class, "rightFront");
        leftFront = aHwMap.get(DcMotor.class, "leftFront");
        rightRear = aHwMap.get(DcMotor.class, "rightRear");
        leftRear = aHwMap.get(DcMotor.class, "leftRear");
        pinger = aHwMap.get(Servo.class, "pinger");
        
        imu = aHwMap.get(BNO055IMU.class, "imu");
        xEncoder = aHwMap.get(AnalogSensor.class, "xEncoder");
        yEncoder = aHwMap.get(AnalogSensor.class, "yEncoder");
        
    }
    
    
    
    // Stations the robot in current position
    public void brake() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    
    // Moves the left and right side motors
    public void steer(double leftSpeed, double rightSpeed) {
        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);

    }
    
    // Moves the robot sideways without turning
    public void strafe(double speed){
        // Positive speed strafes right, negative speed strafes left.
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftRear.setPower(-speed);
        rightRear.setPower(speed);

    }
    
    
    public void updatePos() {
        
        double horizontalEncoderValue = xEncoder.readRawVoltage();
        double verticalEncoderValue = yEncoder.readRawVoltage();
        worldAngle_rad = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        
        double newWorldX = verticalEncoderValue*Math.cos(worldAngle_rad);
        double newWorldY = verticalEncoderValue*Math.sin(worldAngle_rad);
        worldXPosition = newWorldX;
        worldYPosition = newWorldY;
        
        
    }
    
    
    // Extends pinger to its maximum length
    public void pingerOut() {
        pinger.setPosition(0.25);
        
    }
    
    
    // Retracts pinger into the robot
    public void pingerIn() {
        pinger.setPosition(0);
    
    }



}
