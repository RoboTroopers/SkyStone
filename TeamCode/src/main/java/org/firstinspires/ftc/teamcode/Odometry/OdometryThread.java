package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.Utilities.MiscUtil;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class OdometryThread implements Runnable {
    
    public Robot robot;
    public int sleepTime = 75;
    
    public OdometryThread(Robot arobot) {
        robot = arobot;
        
    }
    
    
    public void run() {
        robot.odometry.updatePos();
        MiscUtil.pause(sleepTime);
        
    }
    
}
