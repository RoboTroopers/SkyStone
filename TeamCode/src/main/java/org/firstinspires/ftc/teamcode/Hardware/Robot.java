package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OdometrySystem.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.AdvancedMovement;

import static java.lang.Math.toRadians;


public class Robot {
    
    
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Sensing sensing = new Sensing();
    
    public Odometry odometry = new Odometry();
    
    public AdvancedMovement advancedMovement = new AdvancedMovement(this);


    public void setPosition(double xInches, double yInches, double degrees) {
        odometry.setPosition(this, xInches, yInches, toRadians(degrees));
        
    }
    
    public void initHardware (HardwareMap aHwMap) {
        
        driveTrain.initHardware(aHwMap);
        intake.initHardware(aHwMap);
        sensing.initHardware(aHwMap);
        
    }
    
    
    
}