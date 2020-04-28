package org.firstinspires.ftc.teamcode.Hardware.OLD.OdometrySystem;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;
import org.firstinspires.ftc.teamcode.Util.MiscUtil;

public class OdometryThread implements Runnable {

    private OLDRobot robot;
    private int sleepTime = 75;
    
    public OdometryThread(OLDRobot theRobot) {
        robot = theRobot;
    }

    
    public void run() {
        robot.odometry.updatePosition();
        MiscUtil.pause(sleepTime);
    }
    
    
}
