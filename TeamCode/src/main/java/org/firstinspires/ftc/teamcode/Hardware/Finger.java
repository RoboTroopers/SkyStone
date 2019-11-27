package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Finger {


    public Servo finger;
    
    private double FINGER_DOWN_POS = 0.1;
    public double FINGER_UP_POS = 0.5;

    public enum States { DOWN, UP }
    
    
    public void specifyPositions(double down, double up) {
        FINGER_DOWN_POS = down;
        FINGER_UP_POS = up;
        
    }
    
    
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
