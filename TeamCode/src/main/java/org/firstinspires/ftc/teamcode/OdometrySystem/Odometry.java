package org.firstinspires.ftc.teamcode.OdometrySystem;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Utilities.DriveConstants.ticksToInches;


public class Odometry {
    
    private Robot robot;
    
    
    public double worldXPosition = 0;
    public double worldYPosition = 0;


    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }


    public double getXInches(){
        return ticksToInches(worldXPosition);
    }

    public double getYInches() {
        return ticksToInches(worldYPosition);
    }
    
    
    public double horizontalEncoderLast;
    public double verticalEncoderLast;


    
    public void setPosition(Robot theRobot, double startXInches, double startYInches, double startWorldAngle_rad) {
        robot = theRobot;
        worldXPosition = startXInches;
        worldYPosition = startYInches;
        robot.sensing.worldAngle_rad = startWorldAngle_rad;
        
    }


    public void updatePos() {

        robot.sensing.updateAngle();
        double xEncoderChange = robot.sensing.getHorizontalEncoder() - horizontalEncoderLast;
        double yEncoderChange = robot.sensing.getVerticalEncoder() - verticalEncoderLast;

        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = yEncoderChange * Math.cos(robot.sensing.worldAngle_rad);
        double forwardShiftY = yEncoderChange * Math.sin(robot.sensing.worldAngle_rad);

        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = xEncoderChange*Math.cos(robot.sensing.worldAngle_rad);
        double strafeShiftY = yEncoderChange*Math.sin(robot.sensing.worldAngle_rad);

        worldXPosition += forwardShiftX + strafeShiftX;
        worldYPosition += forwardShiftY + strafeShiftY;

        horizontalEncoderLast = robot.sensing.getHorizontalEncoder();
        verticalEncoderLast = robot.sensing.getVerticalEncoder();


    }
}
