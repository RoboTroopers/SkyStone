package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Finger {


    public Servo finger;

    public final double FINGER_DOWN_POS = 0.1;
    public final double FINGER_UP_POS = 0.55;

    public enum States { DOWN, UP }


    public void initHardware(HardwareMap aHwMap, String name) {
        finger = aHwMap.get(Servo.class, name);

    }


    public void down() {
        finger.setPosition(FINGER_DOWN_POS);
    }


    public void up() {
        finger.setPosition(FINGER_UP_POS);
    }


    public States getState() {

        States fingerState;

        if (finger.getPosition() == FINGER_UP_POS) {
            fingerState = States.UP;

        } else {
            fingerState = States.DOWN;
        }
        
        return fingerState;
    }


}
