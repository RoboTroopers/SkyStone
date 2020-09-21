package org.firstinspires.ftc.opmodes;

import org.firstinspires.ftc.Bot;
import org.firstinspires.ftc.util.GamepadPressTracker;

public class TeleOp extends Bot
{
    private static final double threshold = 0.05;
    private static final double turnSpeed = 0.5;

    private GamepadPressTracker gamepadPressTracker1;
    private GamepadPressTracker gamepadPressTracker2;

    @Override
    public void init()
    {
        super.init();
        gamepadPressTracker1 = new GamepadPressTracker(gamepad1);
        gamepadPressTracker2 = new GamepadPressTracker(gamepad2);
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
            movementTurn = -gamepad1.right_stick_x * turnSpeed;
        }

        applyMovement(movementY, movementX, movementTurn);
    }
}
