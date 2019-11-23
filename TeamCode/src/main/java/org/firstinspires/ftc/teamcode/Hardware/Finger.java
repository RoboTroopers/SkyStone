package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Finger {


    public Servo finger;

    public final double FINGER_DOWN_POS = 0.5;
    public final double FINGER_UP_POS = 0;

    public enum States { DOWN, UP }



    public void initHardware(HardwareMap aHwMap) {
        finger = aHwMap.get(Servo.class, "finger");
        finger.setDirection(Servo.Direction.REVERSE);

    }


    public void down() {
        finger.setPosition(FINGER_DOWN_POS);
    }

    public void up() {
        finger.setPosition(FINGER_UP_POS);
    }


    public States getState() {

        States fingerState;

        if (finger.getPosition() == FINGER_DOWN_POS) {
            fingerState = States.DOWN;
        } else {
            fingerState = States.UP;
        }
        
        return fingerState;
    }


}
