package org.firstinspires.ftc.teamcode.Util;


public class MiscUtil {

    public static void pause(int sleepTime) {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public static boolean anyTrue(boolean[] array) {
        for(boolean b: array)
            if (b) return true;
        return false;
    }

}
