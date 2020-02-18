package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CustomTelemetry;


@TeleOp(name = "Color Sensor Test")
//@Disabled
public class DistanceSensorTest extends OpMode {

    private Robot robot = new Robot(this);


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.sensors.lineSensor.enableLed(true);
    }



    @Override
    public void loop() {
        CustomTelemetry.sensorData(robot, telemetry);
        telemetry.update();
    }



}
