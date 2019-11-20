package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {


    public DcMotor leftIntake;
    public DcMotor rightIntake;

    public enum Directions {

        REST,
        SUCK,
        BLOW,

    }




    public void initHardware(HardwareMap aHwMap) {

        leftIntake = aHwMap.get(DcMotor.class, "leftIntake");
        rightIntake = aHwMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

    }


    // Set intake speed to suck in skystone
    public void setSpeed(double speed) {

        leftIntake.setPower(speed);
        rightIntake.setPower(speed);

    }


    public void stop() {

        leftIntake.setPower(0);
        rightIntake.setPower(0);

    }


    public Directions getDirection() {

        Directions directionState;
        double intakeAvg = rightIntake.getPower()+leftIntake.getPower();

        if (intakeAvg < 0) {
            directionState = Directions.SUCK;

        } else if (intakeAvg > 0) {
            directionState = Directions.BLOW;

        } else {
            directionState = Directions.REST;
        }

        return directionState;
    }



}
