package org.firstinspires.ftc.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadPressTracker
{
    private final Gamepad gamepad;

    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b;
    private int left_bumper, right_bumper;

    public GamepadPressTracker(Gamepad g)
    {
        gamepad = g;
    }

    public void update()
    {
        if (gamepad.x) { ++x; } else { x = 0; }
        if (gamepad.y) { ++y; } else { y = 0; }
        if (gamepad.a) { ++a; } else { a = 0; }
        if (gamepad.b) { ++b; } else { b = 0; }
        if (gamepad.dpad_up) { ++dpad_up; } else { dpad_up = 0; }
        if (gamepad.dpad_down) { ++dpad_down; } else { dpad_down = 0; }
        if (gamepad.dpad_left) { ++dpad_left; } else { dpad_left = 0; }
        if (gamepad.dpad_right) { ++dpad_right; } else { dpad_right = 0; }
        if (gamepad.left_bumper) { ++left_bumper; } else { left_bumper = 0; }
        if (gamepad.right_bumper) { ++right_bumper; } else { right_bumper = 0; }
    }

    public int leftBumperHeld() { return left_bumper; }
    public int rightBumperHeld() {return right_bumper; }

    public boolean dpadUpOnce() { return 1 == dpad_up; }
    public boolean dpadDownOnce() { return 1 == dpad_down; }
    public boolean dpadLeftOnce() { return 1 == dpad_left; }
    public boolean dpadRightOnce() { return 1 == dpad_right; }
    public boolean XOnce() { return 1 == x; }
    public boolean YOnce() { return 1 == y; }
    public boolean AOnce() { return 1 == a; }
    public boolean BOnce() { return 1 == b; }
    public boolean leftBumperOnce() { return 1 == left_bumper; }
    public boolean rightBumperOnce() { return 1 == right_bumper; }

}
