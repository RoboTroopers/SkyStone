package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ppProject.company.Range;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class DriveTrain {
    
    
    public Robot robot;
    
    
    // Motors and servos
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    public DcMotor baseMotors[] = {};


    public void initHardware(HardwareMap aHwMap, Robot theRobot) {
        
        this.robot = theRobot;
        
        leftFront = aHwMap.get(DcMotor.class, "leftFront");
        rightFront = aHwMap.get(DcMotor.class, "rightFront");
        leftRear = aHwMap.get(DcMotor.class, "leftRear");
        rightRear = aHwMap.get(DcMotor.class, "rightRear");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        baseMotors = new DcMotor[] {leftFront, rightFront, leftRear, rightRear};
        
    }



    public void applyMovement(double horizontal, double vertical, double turn) {

        //Moves robot on field horizontally and vertically, rotates by turn

        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double lf_power_raw = vertical + turn - (horizontal*1.5);
        double lr_power_raw = vertical + turn + (horizontal*1.5);
        double rf_power_raw = vertical - turn + (horizontal*1.5);
        double rr_power_raw = vertical - turn - (horizontal*1.5);


        // Find greatest power
        double maxRawPower = Math.max(Math.max(lf_power_raw, rf_power_raw), Math.max(lr_power_raw, rr_power_raw));

        double scaleDownFactor = 1.0;
        if (maxRawPower > 1.0) {
            // Reciprocal of maxRawPower so that when multiplied by factor, maxPower == 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }

        // All motor speeds scaled down (if maxRawPower > 1) but vector is preserved.
        lr_power_raw *= scaleDownFactor;
        rf_power_raw *= scaleDownFactor;
        lr_power_raw *= scaleDownFactor;
        rr_power_raw *= scaleDownFactor;

        // Changes motor powers only if they have changed
        if (leftFront.getPower() != lf_power_raw)
            leftFront.setPower(lf_power_raw);
        if (rightFront.getPower() != rf_power_raw)
            rightFront.setPower(rf_power_raw);
        if (leftRear.getPower() != lr_power_raw)
            leftRear.setPower(lr_power_raw);
        if (rightRear.getPower() != rr_power_raw)
            rightRear.setPower(rr_power_raw);
        
        pause(10);
        
    }
    
    
    
    // Stations the robot in current position
    public void brake() { applyMovement(0, 0, 0); }
    
    
    // Moves all motors at same power
    public void straight(double speed) { applyMovement(0, speed, 0); }

    // Moves all motors at same power
    public void turn(double speed) { applyMovement(0, 0, -speed); }
    
    
    // Moves the left and right side motors separate speeds
    public void turn(double leftSpeed, double rightSpeed) {
        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);
        
    }
    
    
    // Moves the robot sideways without turning, positive speed is right, negative speed is left.
    public void strafe(double speed){ applyMovement(speed, 0, 0); }
    
    
    
    public void setMotorModes(DcMotor.RunMode runMode) {
        
        for (DcMotor motor : baseMotors) {
            motor.setMode(runMode);
        }
        
    }
    
    
    
    public void setTargetPositions(int leftFrontPos, int rightFrontPos, int leftRearPos, int rightRearPos) {
        
        leftFront.setTargetPosition(leftFrontPos);
        rightFront.setTargetPosition(rightFrontPos);
        leftRear.setTargetPosition(leftRearPos);
        rightFront.setTargetPosition(rightRearPos);

    }
    
    
    
    public boolean anyMotorsBusy() { return (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()); }
    
    
    
    public void straightInches(double relativeInches, double speed) {

        int relativePosition = (int)inchesToTicks(relativeInches);

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftFrontPos = relativePosition;
        int rightFrontPos = relativePosition;
        int leftRearPos = relativePosition;
        int rightRearPos = relativePosition;
        
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        
        setTargetPositions(leftFrontPos, rightFrontPos, leftRearPos, rightRearPos);
        straight(speed);
        while (anyMotorsBusy()) {pause(10);}

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brake();

    }
    
    
    
    public void strafeInches(double relativeInches, double rightSpeed) {

        int relativePosition = (int)inchesToTicks(relativeInches);

        int RF_LRSign = (int)(rightSpeed / abs(rightSpeed));
        int LF_RRSign = -RF_LRSign;

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int leftFrontPos = relativePosition*LF_RRSign;
        int rightFrontPos = relativePosition*RF_LRSign;
        int leftRearPos = relativePosition*RF_LRSign;
        int rightRearPos =  relativePosition*LF_RRSign;

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        
        setTargetPositions(leftFrontPos, rightFrontPos, leftRearPos, rightRearPos);
        strafe(rightSpeed);
        while (anyMotorsBusy()) {pause(10);}

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brake();

    }
    
    
    
    public void turnToRad(double absoluteRad, double maxSpeed, double deaccelRate) {

        /**
         * Turns drivetrain to a specific absolute angle in radians.
         * Negative angles are clockwise, positive angles are counterclockwise.
         */

        final double initialRad = robot.sensors.getWorldAngleRad();
        final double initialRelativeRadToAngle = initialRad - absoluteRad;

        double errorRad;

        double rotationAccuracyRange = toRadians(2);
        double minSpeed = 0.01;

        do {
            errorRad = robot.sensors.getWorldAngleRad() - absoluteRad;
            
            double movement_turn = (errorRad/initialRelativeRadToAngle) * deaccelRate; // Speed is greater when error is greater
            movement_turn = Range.clip(movement_turn, minSpeed, maxSpeed); // Deaccel rate only becomes apparent when it isn't being cut off by clip (while deacceling)
            
            applyMovement(0, 0, -movement_turn);
            
        } while (Math.abs(errorRad) > rotationAccuracyRange);

        robot.driveTrain.brake();

    }

    
    
    public void turnToRad(double absoluteRad, double maxSpeed) { turnToRad(absoluteRad, maxSpeed, 3); }
    
    
    
    /*
    public void myGoToPosition(double xInches, double yInches, double movementSpeed, double preferredAngle_rad, double turnSpeed) {
        double accuracyRange = inchesToTicks(0.5);
        double rotAccuracyRange = toRadians(2);
        double x = inchesToTicks(xInches);
        double y = inchesToTicks(yInches);
        double distanceToTarget;
        
        double movement_x;
        double movement_y;
        double movement_turn;

        boolean translationComplete = false;
        boolean rotationComplete = false;

        while (!(translationComplete || rotationComplete)) {
            distanceToTarget = Math.hypot(x -robot.odometry.worldXPosition, y - robot.odometry.worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - robot.odometry.worldYPosition, x - robot.odometry.worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (robot.sensors.getWorldAngleRad() - toRadians(90)));

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
        double desiredAngle_rad = robot.sensors.getWorldAngleRad();
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
