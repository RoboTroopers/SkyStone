package org.firstinspires.ftc.teamcode.OdometrySystem;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Unused.FieldPosition;

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
    
    
    public Odometry(Robot theRobot) {
        
        robot = theRobot;
        
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
        
        double horizontalEncoderChange = robot.sensors.getHorizontalEncoder() - horizontalEncoderLast;
        double verticalEncoderChange = robot.sensors.getVerticalEncoder() - verticalEncoderLast;
        
        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = verticalEncoderChange * Math.cos(robot.sensors.getWorldAngleRad());
        double forwardShiftY = verticalEncoderChange * Math.sin(robot.sensors.getWorldAngleRad());
        
        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = horizontalEncoderChange*Math.cos(robot.sensors.getWorldAngleRad());
        double strafeShiftY = horizontalEncoderChange*Math.sin(robot.sensors.getWorldAngleRad());
        
        worldXPosition += forwardShiftX + strafeShiftX;
        worldYPosition += forwardShiftY + strafeShiftY;
        
        horizontalEncoderLast = robot.sensors.getHorizontalEncoder();
        verticalEncoderLast = robot.sensors.getVerticalEncoder();
        
        
        
    }
}
