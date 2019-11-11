package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    
    
    public DcMotor leftPulley;
    public DcMotor rightPulley;
    
    public Servo arm;
    
    public final double ARM_DOWN = 0;
    public final double ARM_UP = 180;
    
    public enum ArmState {
        
        UP,
        DOWN
        
    }
    
    ArmState armState = ArmState.DOWN;
    
    
    
    public void initHardware(HardwareMap aHwMap) {
        
        leftPulley = aHwMap.get(DcMotor.class, "leftPulley");
        rightPulley = aHwMap.get(DcMotor.class, "rightPulley");
        
        arm = aHwMap.get(Servo.class, "arm");
        
    }
    
    
    public void setArm(double deg) {
        
        arm.setPosition(deg);
        
        if (deg > (ARM_UP-ARM_DOWN)/2) {
            armState = ArmState.UP;
        } else {
            armState = ArmState.DOWN;
        }

    }
    
    
}
