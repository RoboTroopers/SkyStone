package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    
    
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    
    public enum DirectionStates {
        REST,
        SUCK,
        BLOW,
        
    }
    
    public DirectionStates currentState = DirectionStates.REST;
    
    
    
    public void initHardware(HardwareMap aHwMap) {
        
        leftIntake = aHwMap.get(DcMotor.class, "leftIntake");
        rightIntake = aHwMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        
    }
    
    
    // Set intake speed to suck in skystone
    public void setSpeed(double speed) {
        
        leftIntake.setPower(speed);
        rightIntake.setPower(speed);
        
        if (speed > 0) {
            currentState = DirectionStates.SUCK;
            
        } else if (speed < 0) {
            currentState = DirectionStates.BLOW;
            
        } else {
            currentState = DirectionStates.REST;
        }
        
    }
    
    
    public void stop() {
        
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        
        currentState = DirectionStates.REST;

    }
    
    
    
}
