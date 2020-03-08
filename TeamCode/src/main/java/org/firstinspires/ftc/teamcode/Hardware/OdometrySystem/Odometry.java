package org.firstinspires.ftc.teamcode.Hardware.OdometrySystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Useless.FieldPosition;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.ticksToInches;


public class Odometry extends HardwareComponent {

    private Robot robot;

    private Thread thread;

    public DcMotor horizontalEncoder;
    //public DcMotor verticalEncoder;
    public DcMotor leftVerticalEncoder;
    public DcMotor rightVerticalEncoder;


    public double worldXPosition = 0;
    public double worldYPosition = 0;
    
    
    public double getXPos(){
        return worldXPosition;
    }
    
    public double getYPos() {
        return worldYPosition;
    }
    
    
    public double getXPosInches() {
        return ticksToInches(worldXPosition);
    }
    
    public double getYPosInches() {
        return ticksToInches(worldYPosition);
    }
    
    
    public double horizontalEncoderLast;
    public double verticalEncoderLast;
    
    
    public Odometry(Robot theRobot, OpMode opMode) {
        super(theRobot, opMode);
        thread = new Thread(new OdometryThread(theRobot));
        thread.start();
    }


    @Override
    public void init(HardwareMap aHwMap) {
        horizontalEncoder = aHwMap.get(DcMotor.class, "horizontalEncoder");
        //verticalEncoder = aHwMap.get(DcMotor.class, "verticalEncoder");
        leftVerticalEncoder = aHwMap.get(DcMotor.class, "leftVerticalEncoder");
        rightVerticalEncoder = aHwMap.get(DcMotor.class, "rightVerticalEncoder");
        resetEncoders();
    }



    public double getHorizontalEncoder() { return horizontalEncoder.getCurrentPosition(); }

    public double getVerticalEncoder() {

        //return verticalEncoder.getCurrentPosition();
        double leftValue = leftVerticalEncoder.getCurrentPosition();
        double rightValue = rightVerticalEncoder.getCurrentPosition();
        return (leftValue + rightValue)/2;
    }


    public void resetEncoders() {
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //verticalEncoderverticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //verticalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVerticalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setPosition(FieldPosition fieldPos) {
        worldXPosition = fieldPos.fieldXTicks;
        worldYPosition = fieldPos.fieldYTicks;
    }
    
    
    public void setPositionTicks(double startX, double startY) {
        worldXPosition = startX;
        worldYPosition = startY;
    }


    public void setPositionInches(double startXInches, double startYInches) {
        worldXPosition = ticksToInches(startXInches);
        worldYPosition = ticksToInches(startYInches);
    }
    
    
    public void updatePosition() {
        double horizontalEncoderChange = getHorizontalEncoder() - horizontalEncoderLast;
        double verticalEncoderChange = getVerticalEncoder() - verticalEncoderLast;
        
        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = verticalEncoderChange * Math.cos(robot.sensors.getWorldAngleRad());
        double forwardShiftY = verticalEncoderChange * Math.sin(robot.sensors.getWorldAngleRad());
        
        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = horizontalEncoderChange*Math.cos(robot.sensors.getWorldAngleRad());
        double strafeShiftY = horizontalEncoderChange*Math.sin(robot.sensors.getWorldAngleRad());
        
        worldXPosition += forwardShiftX + strafeShiftX;
        worldYPosition += forwardShiftY + strafeShiftY;
        
        horizontalEncoderLast = getHorizontalEncoder();
        verticalEncoderLast = getVerticalEncoder();
    }


}
