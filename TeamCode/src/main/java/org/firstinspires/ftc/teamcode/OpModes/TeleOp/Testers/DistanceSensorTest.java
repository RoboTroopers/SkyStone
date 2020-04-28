package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;


@TeleOp(name = "Color Sensor Test")
//@Disabled
public class DistanceSensorTest extends OpMode {

    private OLDRobot robot = new OLDRobot(this);


    @Override
    public void init() {

        //robot.sensors.lineSensor.enableLed(true);
    }



    @Override
    public void loop() {
        //CustomTelemetry.sensorData(robot, telemetry);
        telemetry.addData("Pulley height", robot.outtake.getHeight());
        //telemetry.addData("Stone distance", robot.sensors.getStoneDistance());

        telemetry.update();
    }



}
