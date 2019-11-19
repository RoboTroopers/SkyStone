package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;


public class Robot {
    
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;
    
    
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();
    
    public Sensors sensors = new Sensors();
    //public Odometry odometry = new Odometry(this);
    
    
    public void initHardware (HardwareMap aHwMap) {
        
        //get the two expansion hubs themselves
        //revMaster = aHwMap.get(ExpansionHubEx.class,"hub");
        //revSlave = aHwMap.get(ExpansionHubEx.class,"Slave");
        
        
        driveTrain.initHardware(aHwMap, this);
        intake.initHardware(aHwMap);
        sensors.initHardware(aHwMap);
        outtake.initHardware(aHwMap);
        
    }
    
    
    
}