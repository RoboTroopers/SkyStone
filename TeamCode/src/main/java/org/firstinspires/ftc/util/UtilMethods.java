package org.firstinspires.ftc.util;

public class UtilMethods
{
    public static void delay(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch (InterruptedException e)
        { }
    }
}
