package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {

    public final OpMode opMode;
    public final DriveTrain driveTrain;
    public final Intake intake;
    public final Outtake outtake;
    public final Fingers fingers;

    public final Sensors sensors;
    //public Odometry odometry = new Odometry(this);


    public Robot (OpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(this, opMode);
        intake = new Intake(this, opMode);
        outtake = new Outtake(this, opMode);
        fingers = new Fingers(this, opMode);
        sensors = new Sensors(this, opMode);
    }


    public void init(HardwareMap aHwMap) {
        driveTrain.init(aHwMap);
        intake.init(aHwMap);
        outtake.init(aHwMap);
        fingers.init(aHwMap);
        sensors.init(aHwMap);
    }




}