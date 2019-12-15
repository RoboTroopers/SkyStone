package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;



public class CustomTelemetry {

    private Robot robot;
    public Telemetry telemetry;


    public CustomTelemetry(Robot theRobot) {

        robot =  theRobot;
        telemetry = robot.opMode.telemetry;

    }


    public void driveTrainSpeedData() {
        telemetry.addData("Base Speed",
                "\n" +
                        "(%.1f)---(%.1f)\n" +
                        "|          |\n" +
                        "|          |\n" +
                        "(%.1f)---(%.1f)\n",
                robot.driveTrain.leftFront.getPower(),
                robot.driveTrain.rightFront.getPower(),
                robot.driveTrain.leftRear.getPower(),
                robot.driveTrain.rightRear.getPower()
        );
    }



    public void encoderData() {
        telemetry.addData("Base Speed",
                "\n" +
                        "(%.1f)---(%.1f)\n" +
                        "|          |\n" +
                        "|          |\n" +
                        "(%.1f)---(%.1f)\n",
                robot.driveTrain.leftFront.getCurrentPosition(),
                robot.driveTrain.rightFront.getCurrentPosition(),
                robot.driveTrain.leftRear.getCurrentPosition(),
                robot.driveTrain.rightRear.getCurrentPosition()
        );
    }



    public void sensorData() {
        telemetry.addData("Angle", robot.sensors.getWorldAngleDeg());
        telemetry.addData("LineSensor Hue", robot.sensors.getLineSensorHSV()[0]);
        telemetry.addData("LineSensor Sat", robot.sensors.getLineSensorHSV()[1]);
        telemetry.addData("LineSensor Val", robot.sensors.getLineSensorHSV()[2]);
        telemetry.addData("Distance from Sensor", robot.sensors.getDistance());

        telemetry.addData("Distance from Sensor", robot.sensors.stoneFullyIn());
    }


}
