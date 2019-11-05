package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions;
import static org.firstinspires.ftc.teamcode.Utilities.DriveConstants.inchesToTicks;

public class AdvancedMovement {
    
    Robot robot;

    public double movement_x;
    public double movement_y;
    public double movement_turn;
    
    
    public AdvancedMovement(Robot theRobot) {
        robot = theRobot;
        
    }
    

    public void goToPosition(double xInches, double yInches, double movementSpeed, double preferredAngle, double turnSpeed) {
        double accuracyRange = inchesToTicks(0.5);
        double rotAccuracyRange = toRadians(2);
        double x = inchesToTicks(xInches);
        double y = inchesToTicks(yInches);
        double distanceToTarget = Math.hypot(x - robot.odometry.worldXPosition, y - robot.odometry.worldYPosition);
        
        boolean translationComplete = false;
        boolean rotationComplete = false;
        
        while (!(translationComplete || rotationComplete)) {
            distanceToTarget = Math.hypot(x -robot.odometry.worldXPosition, y - robot.odometry.worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - robot.odometry.worldYPosition, x - robot.odometry.worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (robot.odometry.worldAngle_rad - toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle;
            movement_turn = Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;
            
            translationComplete = (Math.abs(distanceToTarget) < accuracyRange);
            rotationComplete = (Math.abs(distanceToTarget) < accuracyRange);
            
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

    


    public void turnToDegrees(double degrees, double maxSpeed) {

        // positive degrees == right; negative degrees == left
        double turnDegrees = degrees;

        double targetRange = 0.5; // Distance from the desired angle that is allowed
        double error = turnDegrees; // Distance from the desired range
        double progress; // Degrees the robot has turned already
        double minSpeed = 0.01; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
        double speed; // Actual speed value of the motors

        while (Math.abs(error) > targetRange) {
            progress = turnDegrees-Math.toDegrees(robot.odometry.worldAngle_rad);
            error = progress-turnDegrees;

            speed = ((1-(progress/turnDegrees))*maxSpeed)+minSpeed; // Speed starts at maximum and approaches minimum as the gyro value approaches the desired angle. It deccelerates for precision and accuray.
            speed = Range.clip(speed, minSpeed, maxSpeed);

            if (turnDegrees < 0)
                robot.driveTrain.steer(-speed, speed);
            else
                robot.driveTrain.steer(speed, -speed);
        }

        robot.driveTrain.brake();
    }


}
