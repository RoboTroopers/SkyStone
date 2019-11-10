package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OdometrySystem.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.AdvancedMovement;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Utilities.DriveConstants.inchesToTicks;


public class Robot {
    
    
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Pinger pinger = new Pinger();
    
    public Sensing sensing = new Sensing();
    public Odometry odometry = new Odometry();
    
    public AdvancedMovement advancedMovement = new AdvancedMovement(this);


    public void setPosition(double xInches, double yInches, double degrees) {
        odometry.setPosition(this, inchesToTicks(xInches), inchesToTicks(yInches), toRadians(degrees));
        
    }
    
    public void initHardware (HardwareMap aHwMap) {
        
        driveTrain.initHardware(aHwMap);
        intake.initHardware(aHwMap);
        pinger.initHardware(aHwMap);
        sensing.initHardware(aHwMap);
        
    }
    
    
    
}