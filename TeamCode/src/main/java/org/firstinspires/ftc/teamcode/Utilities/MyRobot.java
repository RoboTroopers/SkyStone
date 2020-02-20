package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Fingers;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

import java.util.ArrayList;


class MyRobot {

    protected final OpMode opMode;

    protected DriveTrain driveTrain;
    protected Intake intake;
    protected Outtake outtake;
    protected Fingers fingers;
    protected Sensors sensors;

    private ArrayList<HardwareComponent> hwComponents = new ArrayList<>();


    public MyRobot(OpMode activeOpMode) {

        opMode = activeOpMode;
/*
        driveTrain = new DriveTrain(this, OpMode);
        intake = new Intake(this, OpMode);
        outtake = new Outtake(this, OpMode);
        fingers = new Fingers(this, OpMode);
        sensors = new Sensors(this, OpMode);


        hwComponents.add(driveTrain);
        hwComponents.add(intake);
        hwComponents.add(outtake);
        hwComponents.add(fingers);
        hwComponents.add(sensors);
*/
        for (HardwareComponent hwComponent: hwComponents) {
            hwComponent.init(opMode.hardwareMap);
        }

    }



}
