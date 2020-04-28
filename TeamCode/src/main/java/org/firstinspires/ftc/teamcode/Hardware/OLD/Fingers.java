package org.firstinspires.ftc.teamcode.Hardware.OLD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Fingers extends HardwareComponent {


    public Servo pepeSMASH;

    public final double PEPESMASH_IN_POS = 0.1;
    public final double PEPESMASH_OUT_POS = 0.55;


    public enum States { IN, OUT }


    public Fingers(OLDRobot theRobot, OpMode opMode) {
        super(theRobot, opMode);
    }


    @Override
    public void init(HardwareMap aHwMap) {
        pepeSMASH = aHwMap.get(Servo.class, "pepeSMASH");
    }


    public void pepeSMASHIn() {
        pepeSMASH.setPosition(PEPESMASH_IN_POS);
    }

    public void pepeSMASHOut() {
        pepeSMASH.setPosition(PEPESMASH_OUT_POS);
    }


    public States getPepeSMASHState() {
        States fingerState;

        if (pepeSMASH.getPosition() == PEPESMASH_OUT_POS) {
            fingerState = States.IN;

        } else {
            fingerState = States.OUT;
        }

        return fingerState;
    }


}
