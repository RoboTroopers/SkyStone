package org.firstinspires.ftc.teamcode.OdometrySystem;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.MiscUtil;

public class OdometryThread implements Runnable {
    
    private Robot robot;
    private int sleepTime = 75;
    
    public OdometryThread(Robot theRobot) {
        robot = theRobot;
        
    }
    
    
    public void run() {
        //robot.odometry.updatePosition();
        
        MiscUtil.pause(sleepTime);
        
    }
    
    
}
