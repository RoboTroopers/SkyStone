package org.firstinspires.ftc.teamcode.Utilities;


public class MiscUtil {
    
    
    public static void pause(int sleepTime) {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    
    
}
