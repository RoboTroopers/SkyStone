package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.MyMath;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class Outtake extends HardwareComponent {


    public DcMotor leftPulley;
    public DcMotor rightPulley;

    public CRServo leftTail; //TODO: use only one tail
    public CRServo rightTail; // <-- Delete this boi


    public Servo claw;

    public final double CLAW_OPEN_POS = 0.65;
    public final double CLAW_CLOSED_POS = 0.95;


    public DistanceSensor heightSensor;

    public final double HEIGHT_MAX = 2;
    public final double HEIGHT_MID = 5;
    public final double HEIGHT_MIN = 10;


    public Outtake(Robot theRobot, OpMode opMode) {
        super(theRobot, opMode);
    }


    public void init(HardwareMap aHwMap) {
        leftPulley = aHwMap.get(DcMotor.class, "leftPulley");
        rightPulley = aHwMap.get(DcMotor.class, "rightPulley");

        leftTail = aHwMap.get(CRServo.class, "leftTail");
        rightTail = aHwMap.get(CRServo.class, "rightTail");

        claw = aHwMap.get(Servo.class, "claw");
        heightSensor = aHwMap.get(DistanceSensor.class, "heightSensor");

    }


    public void setPulleySpeed(double speed) {
        leftPulley.setPower(speed);
        rightPulley.setPower(speed);
    }


    public void stopPulley() {
        setPulleySpeed(0);
    }

    public double getPulleySpeed() {
        return (leftPulley.getPower() + rightPulley.getPower())/2;
    }


    public void setTailSpeed(double speed) {
        leftTail.setPower(speed);
    }

    public void stopTail() {
        setTailSpeed(0);
    }


    public void thrustTail() {
        double speed = 0.5;
        leftTail.setPower(speed);
        rightTail.setPower(speed);
    }


    public void retractTail() {

        double speed = -0.5;
        leftTail.setPower(speed);
        rightTail.setPower(speed);

    }


    // Set the arm to certain positions and set the wrist position to compensate, keeping the claw parallel to the ground
    public void thrustTailAuto() {
        thrustTail();
        pause(500);

        leftTail.setPower(0);
        rightTail.setPower(0);
    }


    public void retractTailAuto() {
        retractTail();
        pause(500);

        leftTail.setPower(0);
        rightTail.setPower(0);
    }


    public double getElbow() {
        return (leftTail.getPower()+ rightTail.getPower())/2;
    }


    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POS);
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POS);
    }

    public double getClawPos() {
        return claw.getPosition();
    }


    public double getHeight() {
        return heightSensor.getDistance(INCH);
    }


    public void liftToHeight(double height, double maxSpeed) {
        final double initialError = height-getHeight();
        final double minSpeed = 0.075;
        final double acceptableRange = 0.5;

        double error = initialError;

        while (abs(error) < acceptableRange && !opModeStopRequested()) {
            error = height-getHeight();
            double errorRatio = error/initialError;
            // Set speed proportional to error if error is between minSpeed and maxSpeed.
            setPulleySpeed(MyMath.clamp(errorRatio, maxSpeed, minSpeed));
        }

        stopPulley();
    }



    public void liftToMax() {
        liftToHeight(HEIGHT_MAX, 0.1);
    }

    public void resetToMid() {
        liftToHeight(HEIGHT_MID, 0.1);
    }

    public void lowerToMin() {
        liftToHeight(HEIGHT_MIN, 0.1);
    }



    public void autoLift() {
        resetToMid();
        closeClaw();
        pause(400);
        liftToMax();
        thrustTailAuto();
    }


    public void autoDeposit() {
        lowerToMin();
        openClaw();
        pause(300);
        liftToMax();
        retractTailAuto();
        pause(300);
        resetToMid();
    }


    public void fullAutoLiftDeposit() {
        autoLift();
        autoDeposit();
    }

/*
    public void liftStone() {
        armMid();

        pause(1500);

        armIn();
        closeClaw();
        pause(1500);
        armOut();
    }

    public void depositStone() {
        liftStone();

        openClaw();
        pause(1000);
        armMid();
    }*/


}
