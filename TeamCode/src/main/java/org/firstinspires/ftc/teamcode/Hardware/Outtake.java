package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class Outtake {


    public DcMotor pulley;

    public Servo arm;

    public final double ARM_IN_POS = 0.7;
    public final double ARM_MID_POS = 0.4;
    public final double ARM_OUT_POS = 0;

    public Servo wrist;

    public final double WRIST_IN_POS = 0.9;
    public final double WRIST_MID_POS = 0.7;
    public final double WRIST_OUT_POS = 0;


    public Servo claw;

    public final double CLAW_OPEN_POS = 0.65;
    public final double CLAW_CLOSED_POS = 0.95;


    //public DistanceSensor pulleySensor;


    public void initHardware(HardwareMap aHwMap) {

        pulley = aHwMap.get(DcMotor.class, "pulley");
        //pulleySensor = aHwMap.get(DistanceSensor.class, "pulleySensor");

        arm = aHwMap.get(Servo.class, "arm");

        wrist = aHwMap.get(Servo.class, "wrist");
        //wrist.setDirection(Servo.Direction.REVERSE);

        claw = aHwMap.get(Servo.class, "claw");

    }


    public void setPulleySpeed(double speed) {
        pulley.setPower(speed);
    }

    public void stopPulley() {
        pulley.setPower(0);
    }

    public double getPulleySpeed() {
        return pulley.getPower();
    }

    /*
    public double getPulleyHeight() {

        double height = pulleySensor.getDistance(DistanceUnit.INCH);
        return height;
    }*/


    // Set the arm to certain positions and set the wrist position to compensate, keeping the claw parallel to the ground
    public void armMid() {

        arm.setPosition(ARM_MID_POS);
        wrist.setPosition(WRIST_MID_POS);
    }


    public void armIn() {

        arm.setPosition(ARM_IN_POS);
        wrist.setPosition(WRIST_IN_POS);
    }


    public void armOut() {

        arm.setPosition(ARM_OUT_POS);
        wrist.setPosition(WRIST_OUT_POS);
    }


    public double getArmPos() {
        return arm.getPosition();
    }


    public double getWristPos() {
        return wrist.getPosition();
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

    }



}
