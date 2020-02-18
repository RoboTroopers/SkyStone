package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;



public class CustomTelemetry {

    public static void speedData(Robot robot, Telemetry telemetry) {
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



    public static void encoderData(Robot robot, Telemetry telemetry) {
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


    public static void sensorData(Robot robot, Telemetry telemetry) {
        telemetry.addData("Angle", robot.sensors.getWorldAngleDeg());
        telemetry.addData("Is over Line", robot.sensors.isOverLine());
        telemetry.addData("Red side", robot.sensors.isRedSide());
        telemetry.addData("Distance from Sensor", robot.sensors.getDistance());
    }


}
