package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {

    public DcMotor leftIntake;
    public DcMotor rightIntake;
    
    public Servo pinger;
    
    public final double PINGER_MIN_POS = 0;
    public final double PINGER_MAX_POS = 50;


    public void initHardware(HardwareMap aHwMap) {
        pinger = aHwMap.get(Servo.class, "pinger");

        leftIntake = aHwMap.get(DcMotor.class, "leftIntake");
        rightIntake = aHwMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
    }


    // Extends pinger to its maximum length
    public void pingerOut() {
        pinger.setPosition(0.25);

    }


    // Retracts pinger into the robot
    public void pingerIn() {
        pinger.setPosition(0);

    }


    // Retracts pinger into the robot
    public void startSucc(double speed) {
        leftIntake.setPower(speed);

    }


    // Retracts pinger into the robot
    public void stopSucc() {
        leftIntake.setPower(0);

    }

}
