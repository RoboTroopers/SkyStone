package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class OdometryThread implements Runnable {
    
    public Robot robot;
    public int sleepTime = 75;
    
    public OdometryThread(Robot arobot) {
        robot = arobot;
        
    }
    
    
    public void run() {
        robot.odometry.updatePos();
        MathFunctions.pause(sleepTime);
        
    }
    
}
