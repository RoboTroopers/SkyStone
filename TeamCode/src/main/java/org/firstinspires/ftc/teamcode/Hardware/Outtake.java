package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;

public class Outtake {
    
    
    public DcMotor pulley;


    public final double ARM_IN_POS = 0;
    public final double ARM_OUT_POS = 0.5;

    public Servo leftArm;
    public Servo rightArm;


    public Servo wrist;

    public Servo claw;

    public final double CLAW_OPEN_POS = 0;
    public final double CLAW_CLOSED_POS = 0.3;


    //public DistanceSensor pulleySensor;


    public void initHardware(HardwareMap aHwMap) {

        pulley = aHwMap.get(DcMotor.class, "leftPulley");
        //pulleySensor = aHwMap.get(DistanceSensor.class, "pulleySensor");

        wrist = aHwMap.get(Servo.class, "wrist");
        claw = aHwMap.get(Servo.class, "claw");

    }


    public void setPulleySpeed(double speed) { pulley.setPower(speed); }

    public void stopPulley() { pulley.setPower(0); }

    public double getPulleySpeed() { return pulley.getPower(); }


    /*
    public double getPulleyHeight() {

        double height = pulleySensor.getDistance(DistanceUnit.INCH);
        return height;

    }*/


    public void armIn() {

        leftArm.setPosition(ARM_IN_POS);
        rightArm.setPosition(ARM_IN_POS);

        wrist.setPosition(ARM_OUT_POS);
    }


    public void armOut() {

        leftArm.setPosition(ARM_IN_POS);
        rightArm.setPosition(ARM_IN_POS);

        wrist.setPosition(ARM_OUT_POS);
    }


    public double getArmPos() { return (leftArm.getPosition()+rightArm.getPosition())/2; }


    public double getWristPos() { return wrist.getPosition(); }


    public void closeClaw() { claw.setPosition(CLAW_CLOSED_POS); }

    public void openClaw() { claw.setPosition(CLAW_OPEN_POS); }
    
    public double getClawPos() { return claw.getPosition(); }


    public void dropStone() {

        closeClaw();
        armOut();
        openClaw();
        pause(1000);

        armIn();
    }

    
}
