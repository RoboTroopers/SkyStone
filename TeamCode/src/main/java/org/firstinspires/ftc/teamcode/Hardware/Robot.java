package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.AdvancedMovement;
import org.openftc.revextensions2.ExpansionHubEx;


public class Robot {
    
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;
    
    
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    //public Outtake outtake = new Outtake();
    //public Accessories accessories = new Accessories();
    
    public Sensing sensing = new Sensing();
    //public Odometry odometry = new Odometry(this);
    
    public AdvancedMovement advancedMovement = new AdvancedMovement(this);
    
    
    public void initHardware (HardwareMap aHwMap) {
        
        //get the two expansion hubs themselves
        revMaster = aHwMap.get(ExpansionHubEx.class,"hub");
        revSlave = aHwMap.get(ExpansionHubEx.class,"slave");
        
        driveTrain.initHardware(aHwMap);
        intake.initHardware(aHwMap);
        //accessories.initHardware(aHwMap);
        sensing.initHardware(aHwMap);
        //outtake.initHardware(aHwMap);
        
    }
    
    
    
}