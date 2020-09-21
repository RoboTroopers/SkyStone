package org.firstinspires.ftc;

/**
 * Sets the desired
 */

public class Auton extends Bot
{
    // Distance from target position and angle necessary to move on to next movement stage.
    private static final double posErrorThreshold = 3;
    private static final double angleErrorThreshold = 0.5;

    private double targetX;
    private double targetY;
    private double xSpeed;
    private double ySpeed;

    private double targetAngle;
    private double turnSpeed;

    private int currentStage;


    @Override
    public void init()
    {
        super.init();
    }

    @Override
    public void loop()
    {
        super.loop();
        if (currentStage == 0)
        {
            targetX = 100;
            targetY = 50;

            xSpeed = 0.2;
            ySpeed = 0.2;

            targetAngle = 90;
            turnSpeed = 0.3;
        }

        if (currentStage == 1)
        {
            targetX = 200;
            targetY = 80;

            xSpeed = 0.2;
            ySpeed = 0.2;

            targetAngle = 30;
            turnSpeed = 0.3;
        }

        glideToPos(targetX, targetY, xSpeed, ySpeed, targetAngle, turnSpeed);

        if (Math.abs(targetX - getWorldXPos()) < posErrorThreshold
         && Math.abs(targetY - getWorldYPos()) < posErrorThreshold
         && Math.abs(targetAngle - getWorldAngle()) < angleErrorThreshold)
        {
            currentStage += 1;
            targetX = getWorldXPos();
            targetY = getWorldYPos();
            targetAngle = getWorldAngle();
        }
    }

}
