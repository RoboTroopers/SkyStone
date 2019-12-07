package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.CustomTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


public class Robot {

    //private ExpansionHubEx revMaster;
    //private ExpansionHubEx revSlave;


    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();
    public Fingers fingers = new Fingers();

    public Sensors sensors = new Sensors();
    //public Odometry odometry = new Odometry(this);



    public OpMode opMode;
    public OpModeTypes currentOpModeType;



    public Robot (OpMode theOpMode, OpModeTypes opModeType) {
        opMode = theOpMode;
        currentOpModeType = opModeType;

        //CustomTelemetry customTelemetry = new CustomTelemetry(this);
    }


    public void initHardware (HardwareMap aHwMap) {
        // get the two expansion hubs themselves
        //revMaster = aHwMap.get(ExpansionHubEx.class,"hub");
        //revSlave = aHwMap.get(ExpansionHubEx.class,"Slave");

        driveTrain.initHardware(aHwMap, this);
        intake.initHardware(aHwMap);
        outtake.initHardware(aHwMap);
        fingers.initHardware(aHwMap);

        sensors.initHardware(aHwMap);

    }


}