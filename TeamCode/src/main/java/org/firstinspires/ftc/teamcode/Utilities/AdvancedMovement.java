package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.ppProject.company.Range;

import static java.lang.Math.toRadians;

public class AdvancedMovement {
    
    
    Robot robot;
    
    
    public AdvancedMovement (Robot theRobot) {
        robot = theRobot;
        
    }
    
    /*
    public double movement_x;
    public double movement_y;
    public double movement_turn;
    */
    
    
    public void myTurnToRad(double absoluteRad, double maxSpeed, double deaccelRate) {
        
        /**
         * Turns drivetrain to a specific absolute angle in radians.
         * Positive angles are clockwise, negative angles are counterclockwise.
         */

        double relativeTurnRadians = robot.sensing.worldAngle_rad - absoluteRad;
        double error;

        double rotationAccuracyRange = toRadians(2);
        double minSpeed = 0.01;
        
        do {
            error = robot.sensing.worldAngle_rad - absoluteRad;
    
            double movement_turn = (error/relativeTurnRadians) * deaccelRate; // Speed is greater when error is greater
            movement_turn = Range.clip(movement_turn, minSpeed, maxSpeed); // Deaccel rate only becomes apparent when it isn't being cut off by clip (while deacceling)
            robot.driveTrain.applyMovement(0, 0, movement_turn);
            
        } while (Math.abs(error) > rotationAccuracyRange);
        
        robot.driveTrain.brake();
        
    }
    
    
    
    public void myTurnToDeg(double absoluteDeg, double maxSpeed, double deaccelRate) {
        /**
         * Turns drivetrain to a specific absolute angle in degrees.
         * Positive angles are clockwise, negative angles are counterclockwise.
         */
        
        myTurnToRad(toRadians(absoluteDeg), maxSpeed, deaccelRate);
        
    }
    
    
    
    /*
    public void myGoToPosition(double xInches, double yInches, double movementSpeed, double preferredAngle_rad, double turnSpeed) {
        double accuracyRange = inchesToTicks(0.5);
        double rotAccuracyRange = toRadians(2);
        double x = inchesToTicks(xInches);
        double y = inchesToTicks(yInches);
        double distanceToTarget;

        boolean translationComplete = false;
        boolean rotationComplete = false;

        while (!(translationComplete || rotationComplete)) {
            distanceToTarget = Math.hypot(x -robot.odometry.worldXPosition, y - robot.odometry.worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - robot.odometry.worldYPosition, x - robot.odometry.worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (robot.sensing.worldAngle_rad - toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
            
            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            
            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;
            
            double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle_rad;
            movement_turn = Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            translationComplete = (Math.abs(distanceToTarget) < accuracyRange);
            rotationComplete = (Math.abs(distanceToTarget) < rotAccuracyRange);

            if (translationComplete) {
                movement_x = 0;
                movement_y = 0;
                
            }
            
            if (rotationComplete) {
                movement_turn = 0;
                
            }
            
            robot.driveTrain.applyMovement(movement_x, movement_y, movement_turn);
        }
        robot.driveTrain.brake();
    }
    
    
    // Stations the robot in current position
    public void brakePID() {
        double desiredXInches = robot.odometry.getXPosInches();
        double desiredYInches = robot.odometry.getYPosInches();
        double desiredAngle_rad = robot.sensing.worldAngle_rad;
        robot.driveTrain.brake();
        myGoToPosition(desiredXInches, desiredYInches, 0.1, desiredAngle_rad, 0.1);
        
    }


    public void turnPID(double desiredRadians, double turnSpeed) {
        double desiredXInches = robot.odometry.getXPosInches();
        double desiredYInches = robot.odometry.getYPosInches();
        myGoToPosition(desiredXInches, desiredYInches, 0.1, desiredRadians, turnSpeed);
        
    }
    */
    
}
