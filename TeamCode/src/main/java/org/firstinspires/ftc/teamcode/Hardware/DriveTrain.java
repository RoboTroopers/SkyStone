package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ppProject.treamcode.ppMode;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Utilities.GamerMath.angleWrapDeg;
import static org.firstinspires.ftc.teamcode.Utilities.GamerMath.castRound;
import static org.firstinspires.ftc.teamcode.Utilities.GamerMath.clamp;
import static org.firstinspires.ftc.teamcode.Utilities.GamerMath.clampSigned;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;



public class DriveTrain extends HardwareComponent {

    public Robot robot;

    // Motors and servos
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;


    public DriveTrain(Robot theRobot, OpMode opMode)  {
        super(theRobot, opMode);
    }


    public void init(HardwareMap aHwMap) {

        leftFront = aHwMap.get(DcMotor.class, "leftFront");
        leftRear = aHwMap.get(DcMotor.class, "leftRear");
        rightFront = aHwMap.get(DcMotor.class, "rightFront");
        rightRear = aHwMap.get(DcMotor.class, "rightRear");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

    }


    public void applyMovement(double straight, double strafe, double turn) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Moves robot on field forward and sideways, rotates by turn
        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double lf_power_raw = straight - turn - (strafe*1.5);
        double lr_power_raw = straight - turn + (strafe*1.5);
        double rf_power_raw = straight + turn + (strafe*1.5);
        double rr_power_raw = straight + turn - (strafe*1.5);

        // Find greatest power
        double maxRawPower = Math.max(Math.max(abs(lf_power_raw), abs(rf_power_raw)), Math.max(abs(lr_power_raw), abs(rr_power_raw)));

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

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


    // Moves all motors at same power
    public void straight(double speed) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        applyMovement(speed, 0, 0);
    }


    // Moves all motors at same power
    public void turn(double speed) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        applyMovement(0, 0, -speed);
    }


    // Moves the left and right side motors separate speeds
    public void steer(double leftSpeed, double rightSpeed) {

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);

    }


    // Moves the robot sideways without turning, positive speed is right, negative speed is left.
    public void strafe(double speed){
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //applyMovement(0, speed, 0);

        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(-speed);
    }



    public void setMotorModes(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);

    }


    public void resetEncoders() {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public int getAvgMotorPosAbs() {
        int motorAvgPower =
            abs(leftFront.getCurrentPosition()) +
            abs(leftRear.getCurrentPosition()) +
            abs(rightFront.getCurrentPosition()) +
            abs(rightRear.getCurrentPosition());
        return castRound(motorAvgPower/4.0);
    }



    public boolean anyMotorsBusy() {
        return (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy());
    }



    public void straightInches(double inches,
                               double maxSpeed) {

        brake();
        resetEncoders();

        double minSpeed = 0.09;
        double accelRate = 1;
        double deaccelRate = 1;

        final int targetPos = inchesToTicks(inches);
        final int initialError = targetPos - getAvgMotorPosAbs();
        final int acceptableError = 6;

        double error = targetPos - getAvgMotorPosAbs();

        // While the error is not within acceptable error range, move to the desired position.
        while (abs(error) > acceptableError) {
            error = targetPos - getAvgMotorPosAbs(); // Error = desired - actual.
            double speed = (error/initialError); // Speed is proportional to the error

            // Accel at respective accel rate depending if acceling before halfway point or deacceling after halfway point.
            if (error > initialError/2.0) {
                speed *= accelRate;
            } else {
                speed *= deaccelRate;
            }

            // Keep speed within min and max, no matter if speed is positive or negative.
            speed = clampSigned(speed, minSpeed, maxSpeed);

            applyMovement(speed, 0, 0);

            telemetry.addData("LeftFrontMotor", leftFront.getPower());

            telemetry.addData("Desired distance", targetPos);
            telemetry.addData("Avg pos", getAvgMotorPosAbs());
            telemetry.addData("Distance error", error);
            telemetry.addData("anyMotorsBusy", anyMotorsBusy());

            telemetry.update();
        }
        brake();
    }




    public void backwardInches(double inches,
                               double maxSpeed) {
        brake();
        resetEncoders();

        double minSpeed = 0.09;
        double accelRate = 1;
        double deaccelRate = 1;

        final int targetPos = inchesToTicks(inches);
        final int initialError = targetPos - getAvgMotorPosAbs();
        final int acceptableError = 6;

        double error = targetPos - getAvgMotorPosAbs();

        // While the error is not within acceptable error range, move to the desired position.
        while (abs(error) > acceptableError) {
            error = targetPos - getAvgMotorPosAbs(); // Error = desired - actual.
            double speed = (error/initialError); // Speed is proportional to the error

            // Accel at respective accel rate depending if acceling before halfway point or deacceling after halfway point.
            if (error < initialError/2.0) {
                speed *= accelRate;
            } else {
                speed *= deaccelRate;
            }

            // Keep speed within min and max, no matter if speed is positive or negative.
            speed = clampSigned(speed, minSpeed, maxSpeed);
            applyMovement(-speed, 0, 0);

            telemetry.addData("LeftFrontMotor", leftFront.getPower());

            telemetry.addData("Desired distance", targetPos);
            telemetry.addData("Avg pos", getAvgMotorPosAbs());
            telemetry.addData("Distance error", error);
            telemetry.addData("anyMotorsBusy", anyMotorsBusy());

            telemetry.update();
        }
        brake();
    }




    public void strafeInches(double inches,
                               double maxSpeed) {
        brake();
        resetEncoders();

        double minSpeed = 0.12;
        double accelRate = 1;
        double deaccelRate = 1;

        final int targetPos = inchesToTicks(inches);
        final int initialError = targetPos - getAvgMotorPosAbs();
        final int acceptableError = 8;

        double error = targetPos - leftFront.getCurrentPosition();

        // While the error is not within acceptable error range, move to the desired position.
        while (abs(error) > acceptableError) {
            error = targetPos - leftFront.getCurrentPosition(); // Error = desired - actual.
            double sign = error/abs(error);
            double speed = abs(error/initialError); // Speed is proportional to the error

            // Accel at respective accel rate depending if acceling before halfway point or deacceling after halfway point.
            if (error > initialError/2.0) {
                speed *= accelRate;
            } else {
                speed *= deaccelRate;
            }

            // Keep speed within min and max, no matter if speed is positive or negative.
            speed = clampSigned(speed, minSpeed, maxSpeed);

            applyMovement(0, speed*sign, 0);

            telemetry.addData("LeftFrontMotor", leftFront.getPower());
            telemetry.addData("Desired distance", targetPos);
            telemetry.addData("Avg pos", leftFront.getCurrentPosition());
            telemetry.addData("Distance error", error);
            telemetry.addData("anyMotorsBusy", anyMotorsBusy());

            telemetry.update();
        }
        brake();
    }




    public void turnToDeg(double desiredDeg, double maxSpeed) {

        brake();
        resetEncoders();

        maxSpeed = abs(maxSpeed);
        final double initialAngle = robot.sensors.getWorldAngleDeg();
        final double initialError = angleWrapDeg(desiredDeg - initialAngle); // Error = desired - actual;
        double errorDeg = initialError;

        final double desiredRange = 0.65;
        final double minSpeed = 0.115;
        final double deaccelRate = 2.85;

        while (Math.abs(errorDeg) > desiredRange) {
            errorDeg = angleWrapDeg(desiredDeg - robot.sensors.getWorldAngleDeg());

            double errorSign = errorDeg / abs(errorDeg);
            double errorRatio = abs(errorDeg) / abs(initialError);

            double turnSpeed = errorRatio * maxSpeed * deaccelRate;
            turnSpeed = clamp(turnSpeed, minSpeed, maxSpeed);

            turn(turnSpeed * errorSign);

            telemetry.addData("Angle", robot.sensors.getWorldAngleDeg());
            telemetry.addData("Raw Error (Deg)", desiredDeg - robot.sensors.getWorldAngleDeg());
            telemetry.addData("Error (Deg)", errorDeg);
            telemetry.addData("InitialError", initialError);
            telemetry.addData("ErrorRatio", errorRatio);
            telemetry.addData("TurnSpeed", turnSpeed);
            telemetry.update();
        }

        brake();
    }


}
