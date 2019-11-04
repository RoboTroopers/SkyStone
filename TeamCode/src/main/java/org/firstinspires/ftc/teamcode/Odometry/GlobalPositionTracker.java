package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.Robot.Robot;


public class GlobalPositionTracker {
    
    public Robot robot;


    // Odometry variables
    public double xSpeed = 0;
    public double ySpeed = 0;
    public double turnSpeed = 0;



    public double xEncoderLast;
    public double yEncoderLast;


    
    public void setPosition(Robot theRobot, double startXInches, double startYInches, double startWorldAngle_rad) {
        robot = theRobot;
        robot.worldXPosition = startXInches;
        robot.worldYPosition = startYInches;
        robot.worldAngle_rad = startWorldAngle_rad;
        
    }


    public void updatePos() {

        robot.worldAngle_rad = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

        double xEncoderChange = robot.getXEncoder() - xEncoderLast;
        double yEncoderChange = robot.getYEncoder() - yEncoderLast;

        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = yEncoderChange * Math.cos(robot.worldAngle_rad);
        double forwardShiftY = yEncoderChange * Math.sin(robot.worldAngle_rad);

        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = xEncoderChange*Math.cos(robot.worldAngle_rad);
        double strafeShiftY = yEncoderChange*Math.sin(robot.worldAngle_rad);

        robot.worldXPosition += forwardShiftX + strafeShiftX;
        robot.worldYPosition += forwardShiftY + strafeShiftY;

        xEncoderLast = robot.getXEncoder();
        yEncoderLast = robot.getYEncoder();


    }
}
