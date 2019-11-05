package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.Hardware.Robot;


public class GlobalPositionTracker {
    
    public Robot robot;
    
    
    public double worldXPosition;
    public double worldYPosition;
    public double worldAngle_rad;


    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }
    
    
    public double horizontalEncoderLast;
    public double verticalEncoderLast;


    
    public void setPosition(Robot theRobot, double startXInches, double startYInches, double startWorldAngle_rad) {
        robot = theRobot;
        worldXPosition = startXInches;
        worldYPosition = startYInches;
        worldAngle_rad = startWorldAngle_rad;
        
    }


    public void updatePos() {

        worldAngle_rad = robot.sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

        double xEncoderChange = robot.sensors.getHorizontalEncoder() - horizontalEncoderLast;
        double yEncoderChange = robot.sensors.getVerticalEncoder() - verticalEncoderLast;

        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = yEncoderChange * Math.cos(worldAngle_rad);
        double forwardShiftY = yEncoderChange * Math.sin(worldAngle_rad);

        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = xEncoderChange*Math.cos(worldAngle_rad);
        double strafeShiftY = yEncoderChange*Math.sin(worldAngle_rad);

        worldXPosition += forwardShiftX + strafeShiftX;
        worldYPosition += forwardShiftY + strafeShiftY;

        horizontalEncoderLast = robot.sensors.getHorizontalEncoder();
        verticalEncoderLast = robot.sensors.getVerticalEncoder();


    }
}
