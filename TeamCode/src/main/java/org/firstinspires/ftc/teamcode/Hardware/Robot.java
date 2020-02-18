package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.CustomTelemetry;


public class Robot implements HardwareComponent {

    //private ExpansionHubEx revMaster;
    //private ExpansionHubEx revSlave;

    public DriveTrain driveTrain = new DriveTrain(this);
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();
    public Fingers fingers = new Fingers();

    public Sensors sensors = new Sensors();
    //public Odometry odometry = new Odometry(this);


    public OpMode opMode;



    public Robot (OpMode theOpMode) {
        opMode = theOpMode;
    }


    public void init(HardwareMap aHwMap) {
        // get the two expansion hubs themselves
        //revMaster = aHwMap.get(ExpansionHubEx.class,"hub");
        //revSlave = aHwMap.get(ExpansionHubEx.class,"Slave");

        driveTrain.init(aHwMap);
        intake.init(aHwMap);
        outtake.init(aHwMap);
        fingers.init(aHwMap);

        sensors.init(aHwMap);

    }


}