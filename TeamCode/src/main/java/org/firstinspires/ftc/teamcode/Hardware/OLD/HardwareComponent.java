package org.firstinspires.ftc.teamcode.Hardware.OLD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class HardwareComponent {

    protected OLDRobot robot;
    protected OpMode opMode;
    protected Telemetry telemetry;


    public HardwareComponent(OLDRobot theRobot, OpMode opMode) {
        this.robot = theRobot;
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        //Don't run init until opMode is run, since hwMap == null until then.
    }


    public abstract void init(HardwareMap HwMap);


    protected boolean opModeStopRequested() {
        if (opMode instanceof LinearOpMode) {
            return ((LinearOpMode) opMode).isStopRequested();
        }
        else return false;
    }

}
