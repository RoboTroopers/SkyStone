package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


@TeleOp(name = "Gyro Test")
public class GyroTest extends OpMode {

    /*
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    */
    Robot robot = new Robot(this, OpModeTypes.TELEOP);



    @Override
    public void init() {
        robot.initHardware(hardwareMap);
    }



    @Override
    public void loop() {
        telemetry.addData("Deg", robot.sensors.getWorldAngleDeg());
        telemetry.update();

    }



}
