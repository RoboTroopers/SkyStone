package org.firstinspires.ftc.teamcode.OdometrySystem;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.ticksToInches;


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
    
    
    public double getXPosInches() {
        return ticksToInches(worldXPosition);
    }
    
    public double getYPosInches() {
        return ticksToInches(worldYPosition);
    }
    
    
    public double horizontalEncoderLast;
    public double verticalEncoderLast;
    
    
    
    public void setPosition(Robot theRobot, double startX, double startY, double startWorldAngle_rad) {
        
        robot = theRobot;
        worldXPosition = startX;
        worldYPosition = startY;
        robot.sensing.worldAngle_rad = startWorldAngle_rad;
        
    }


    public void setPositionInches(Robot theRobot, double startXInches, double startYInches, double startWorldAngle_deg) {
        
        robot = theRobot;
        worldXPosition = ticksToInches(startXInches);
        worldYPosition = ticksToInches(startYInches);
        robot.sensing.worldAngle_rad = toRadians(startWorldAngle_deg);
        
    }
    
    
    public void updatePosition() {
        
        double horizontalEncoderChange = robot.sensing.getHorizontalEncoder() - horizontalEncoderLast;
        double verticalEncoderChange = robot.sensing.getVerticalEncoder() - verticalEncoderLast;
        
        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = verticalEncoderChange * Math.cos(robot.sensing.worldAngle_rad);
        double forwardShiftY = verticalEncoderChange * Math.sin(robot.sensing.worldAngle_rad);
        
        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = horizontalEncoderChange*Math.cos(robot.sensing.worldAngle_rad);
        double strafeShiftY = horizontalEncoderChange*Math.sin(robot.sensing.worldAngle_rad);
        
        worldXPosition += forwardShiftX + strafeShiftX;
        worldYPosition += forwardShiftY + strafeShiftY;
        
        horizontalEncoderLast = robot.sensing.getHorizontalEncoder();
        verticalEncoderLast = robot.sensing.getVerticalEncoder();
        
        
        
    }
}
