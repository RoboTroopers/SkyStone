package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pinger {
    

    public Servo pinger;

    public final double PINGER_MIN_POS = 0;
    public final double PINGER_MAX_POS = 50;
    
    
    public enum PositionStates {
        RETRACTED,
        EXTENDED
            
    }

    PositionStates currentState = PositionStates.RETRACTED;
    
    
    public void initHardware(HardwareMap aHwMap) {
        pinger = aHwMap.get(Servo.class, "pinger");
        
    }
    
    
    // Extends pinger to its maximum length
    public void extend() {
        pinger.setPosition(PINGER_MAX_POS);
        currentState = PositionStates.EXTENDED;

    }


    // Retracts pinger into the robot
    public void retract() {
        pinger.setPosition(PINGER_MIN_POS);
        currentState = PositionStates.RETRACTED;

    }



}
