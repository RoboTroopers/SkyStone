package org.firstinspires.ftc.opmodes;

import org.firstinspires.ftc.RoboTrooper;
import org.firstinspires.ftc.util.GamepadTimer;

public class TeleOp extends RoboTrooper
{
    private static final double threshold = 0.05;
    private static final double turnSpeed = 0.5;

    private GamepadTimer gamepad1Timer;
    private GamepadTimer gamepad2Timer;

    @Override
    public void init()
    {
        super.init();
        gamepad1Timer = new GamepadTimer(gamepad1);
        gamepad2Timer = new GamepadTimer(gamepad2);
    }

    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop()
    {
        super.init();
        gamepad1Timer.update();
        gamepad2Timer.update();

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
            movementTurn = -gamepad1.right_stick_x * turnSpeed;
        }

        applyMovement(movementY, movementX, movementTurn);
    }
}
