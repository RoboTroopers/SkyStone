package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class Outtake implements HardwareComponent{


    public DcMotor leftPulley;
    public DcMotor rightPulley;

    public Servo leftElbow;
    public Servo rightElbow;

    public final double ELBOW_IN_POS = 0.7;
    public final double ELBOW_OUT_POS = 0;

    public Servo claw;

    public final double CLAW_OPEN_POS = 0.65;
    public final double CLAW_CLOSED_POS = 0.95;




    public void init(HardwareMap aHwMap) {
        leftPulley = aHwMap.get(DcMotor.class, "leftPulley");
        rightPulley = aHwMap.get(DcMotor.class, "rightPulley");

        leftElbow = aHwMap.get(Servo.class, "leftElbow");
        rightElbow = aHwMap.get(Servo.class, "rightElbow");

        claw = aHwMap.get(Servo.class, "claw");

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



    // Set the arm to certain positions and set the wrist position to compensate, keeping the claw parallel to the ground
    public void zombieArms() {
        leftElbow.setPosition(ELBOW_OUT_POS);
        rightElbow.setPosition(ELBOW_OUT_POS);
    }


    public void submit() {
        leftElbow.setPosition(ELBOW_IN_POS);
        rightElbow.setPosition(ELBOW_IN_POS);
    }


    public double getElbow() {
        return (leftElbow.getPosition()+rightElbow.getPosition())/2;
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

    public void autoLift() {
        closeClaw();
        pause(500);
        setPulleySpeed(0.05);
        zombieArms();
        pause(500);
        stopPulley();
    }


    public void autoDeposit() {
        zombieArms();
        pause(2000);
        openClaw();
        setPulleySpeed(-0.05);
        submit();
        pause(500);
        stopPulley();
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
