package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOp extends Bot
{

    private double threshold = 0.05;

    @Override
    public void init()
    {
        super.init();
    }

    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop()
    {
        super.init();

        double movementY = 0;
        double movementX = 0;
        double movementTurn = 0;

        // Move horizontally and vertically with one joystick, and turn using the other.
        if (Math.abs(gamepad1.left_stick_y) >= threshold)
        {
            movementY = -gamepad1.left_stick_y;
        }

        if (Math.abs(gamepad1.left_stick_x) >= threshold)
        {
            movementX = -gamepad1.left_stick_x;
        }

        if (Math.abs(gamepad1.right_stick_x) >= threshold)
        {
            movementTurn = -gamepad1.right_stick_x * 0.5;
        }

        applyMovement(movementY, movementX, movementTurn);
    }
}
