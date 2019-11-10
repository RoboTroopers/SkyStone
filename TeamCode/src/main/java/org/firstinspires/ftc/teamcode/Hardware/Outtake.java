package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    
    
    public DcMotor leftPulley;
    public DcMotor rightPulley;
    
    //public Servo ;
    
    
    public void initHardware(HardwareMap aHwMap) {
        
        leftPulley = aHwMap.get(DcMotor.class, "leftPulley");
        rightPulley = aHwMap.get(DcMotor.class, "rightPulley");
        
        
    }
    
    
    
}
