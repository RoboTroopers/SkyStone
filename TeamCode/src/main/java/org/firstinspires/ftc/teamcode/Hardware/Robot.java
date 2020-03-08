package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.OdometrySystem.Odometry;


public class Robot {

    public final OpMode opMode;
    public final DriveTrain driveTrain;
    public final Intake intake;
    public final Outtake outtake;
    public final Fingers fingers;

    public final Sensors sensors;
    public Odometry odometry;


    public Robot (OpMode opMode) {
        this.opMode = opMode;

        odometry = new Odometry(this, opMode);

        sensors = new Sensors(this, opMode);
        driveTrain = new DriveTrain(this, opMode);
        intake = new Intake(this, opMode);
        outtake = new Outtake(this, opMode);
        fingers = new Fingers(this, opMode);
    }


    public void init(HardwareMap aHwMap) {
        odometry.init(aHwMap);

        sensors.init(aHwMap);
        driveTrain.init(aHwMap);
        intake.init(aHwMap);
        outtake.init(aHwMap);
        fingers.init(aHwMap);
    }




}