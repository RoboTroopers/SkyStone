package org.firstinspires.ftc.teamcode.OdometrySystem;

import org.firstinspires.ftc.teamcode.Utilities.MiscUtil;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class OdometryThread implements Runnable {

    private Robot robot;
    private int sleepTime = 75;
    
    public OdometryThread(Robot theRobot) {
        robot = theRobot;
        
    }
    
    
    public void run() {
        robot.odometry.updatePos();
        MiscUtil.pause(sleepTime);
        
    }
    
}
