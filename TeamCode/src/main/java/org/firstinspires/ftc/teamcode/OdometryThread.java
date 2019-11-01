package org.firstinspires.ftc.teamcode;

public class OdometryThread implements Runnable {
    
    public Robot robot;
    
    public OdometryThread(Robot arobot) {
        
        robot = arobot;
        
    }
    
    
    
    public void run() {
        robot.updatePos();
        
        
    }
    
}
