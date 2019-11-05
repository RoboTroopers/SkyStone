package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.AdvancedMovement;
import org.firstinspires.ftc.teamcode.Odometry.GlobalPositionTracker;

import static java.lang.Math.toRadians;


public class Robot {
    
    
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Sensors sensors = new Sensors();
    
    public GlobalPositionTracker odometry = new GlobalPositionTracker();
    
    public AdvancedMovement advancedMovement = new AdvancedMovement(this);


    public Robot(double xInches, double yInches, double degrees) {
        
        odometry.setPosition(this, xInches, yInches, toRadians(degrees));
        
    }

    public void initHardware (HardwareMap aHwMap) {
        
        driveTrain.initHardware(aHwMap);
        intake.initHardware(aHwMap);
        sensors.initHardware(aHwMap);
        
        
    }
    
    
    
}