package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;
import org.firstinspires.ftc.teamcode.ppProject.company.Range;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.ticksToInches;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;



public class DriveTrain {

    public Robot robot;


    // Motors and servos
    //DcMotorEx class allows for advanced motor functions like PID methods
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;



    public void initHardware(HardwareMap aHwMap, Robot theRobot) {
        robot = theRobot;

        leftFront = aHwMap.get(DcMotorEx.class, "leftFront");
        leftRear = aHwMap.get(DcMotorEx.class, "leftRear");
        rightFront = aHwMap.get(DcMotorEx.class, "rightFront");
        rightRear = aHwMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        resetEncoders();

    }


    public void applyMovement(double straight, double strafe, double turn) {

        //Moves robot on field forward and sideways, rotates by turn
        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double lf_power_raw = straight + turn - (strafe*1.5);
        double lr_power_raw = straight + turn + (strafe*1.5);
        double rf_power_raw = straight - turn + (strafe*1.5);
        double rr_power_raw = straight - turn - (strafe*1.5);


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
    public void brake() {
        applyMovement(0, 0, 0);
    }


    // Moves all motors at same power
    public void straight(double speed) {
        applyMovement(speed, 0, 0);
    }

    // Moves all motors at same power
    public void turn(double speed) {
        applyMovement(0, 0, -speed);
    }


    // Moves the left and right side motors separate speeds
    public void steer(double leftSpeed, double rightSpeed) {

        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);

    }


    // Moves the robot sideways without turning, positive speed is right, negative speed is left.
    public void strafe(double speed){
        applyMovement(0, speed, 0);
    }



    public void setMotorModes(DcMotorEx.RunMode runMode) {

        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);

    }


    public void resetEncoders() {

        setMotorModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        if (robot.currentOpModeType == OpModeTypes.AUTO) {
            setTargetPos(0, 0, 0, 0);
            setMotorModes(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.opMode.telemetry.addData("RunToPosition", "Set");
            robot.opMode.telemetry.update();
        } else {
            setMotorModes(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        }
    }


    public void setTargetPos(int leftFrontPos, int leftRearPos,  int rightFrontPos, int rightRearPos) {

        leftFront.setTargetPosition(leftFrontPos);
        leftRear.setTargetPosition(leftRearPos);
        rightFront.setTargetPosition(rightFrontPos);
        rightRear.setTargetPosition(rightRearPos);

    }


    public void setTargetPosInches(double leftFrontInches, double leftRearInches, double rightFrontInches, double rightRearInches) {

        leftFront.setTargetPosition((int)inchesToTicks(leftFrontInches));
        leftRear.setTargetPosition((int)inchesToTicks(leftRearInches));
        rightFront.setTargetPosition((int)inchesToTicks(rightFrontInches));
        rightRear.setTargetPosition((int)inchesToTicks(rightRearInches));

    }


    public double getEncoderAvg() {
        double motorAvgPower =
            leftFront.getCurrentPosition()+
            leftRear.getCurrentPosition()+
            rightFront.getCurrentPosition()+
            rightRear.getCurrentPosition();
        return motorAvgPower/4;
    }


    public double getEncoderAvgInches() {
        return ticksToInches(getEncoderAvg());
    }


    public boolean anyMotorsBusy() {
        return (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy());
    }



    public void moveInches(double LFinches, double LRinches,
                           double RFinches, double RRinches,
                           double speed) {
        //moveInches(inches, inches, inches, inches, speed);
        double leftFrontTargetPos = (int)inchesToTicks(LFinches);
        double leftRearTargetPos = (int)inchesToTicks(LRinches);
        double rightFrontTargetPos = (int)inchesToTicks(RFinches);
        double rightRearTargetPos = (int)inchesToTicks(RRinches);

        double averageTargetPos =
                rightRearTargetPos+
                rightRearTargetPos+
                rightRearTargetPos+
                rightRearTargetPos;

        setTargetPos(
                (int)leftFrontTargetPos+leftFront.getCurrentPosition(),
                (int)leftRearTargetPos+leftRear.getCurrentPosition(),
                (int)rightFrontTargetPos+rightFront.getCurrentPosition(),
                (int)rightRearTargetPos+rightRear.getCurrentPosition());

        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);

        while (anyMotorsBusy() && (Math.abs(averageTargetPos) - Math.abs(getEncoderAvg()) > 1)) {
            robot.opMode.telemetry.addData("Desired distance", averageTargetPos);
            robot.opMode.telemetry.addData("Distance covered", getEncoderAvg());

            robot.opMode.telemetry.update();
        }

        brake();
    }


    public void straightInches(double inches, double speed) {
        moveInches(inches, inches, inches, inches, speed);
    }


    public void turnInches(double inches, double speed) {
        moveInches(inches, inches, -inches, -inches, speed);
    }


    public void strafeInches(double inches, double speed) {
        //inches *= 1.5;
        moveInches(inches, -inches, -inches, inches, speed);
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
