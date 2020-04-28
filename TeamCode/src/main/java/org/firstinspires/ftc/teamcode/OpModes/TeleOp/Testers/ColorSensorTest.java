package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;


@TeleOp(name = "Color Sensor Test")
@Disabled
public class ColorSensorTest extends OpMode {

    private OLDRobot robot = new OLDRobot(this);


    @Override
    public void init() {

        robot.sensors.lineSensor.enableLed(true);
    }



    @Override
    public void loop() {
        telemetry.addData("Hue", robot.sensors.lineSensor.argb());
        telemetry.addData("Red", robot.sensors.lineSensor.red());
        telemetry.addData("Green", robot.sensors.lineSensor.green());
        telemetry.addData("Blue", robot.sensors.lineSensor.blue());
        telemetry.addData("Over Line?", robot.sensors.isOverLine());
        telemetry.update();
    }



}
