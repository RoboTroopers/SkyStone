package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class HardwareComponent {

    protected Robot robot;
    protected OpMode opMode;
    protected Telemetry telemetry;


    public HardwareComponent(Robot theRobot, OpMode opMode) {
        this.robot = theRobot;
        this.opMode = opMode;
        telemetry = opMode.telemetry;
    }

    public abstract void init(HardwareMap HwMap);

}
