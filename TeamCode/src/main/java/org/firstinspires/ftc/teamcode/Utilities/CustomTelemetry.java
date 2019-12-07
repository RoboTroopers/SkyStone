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


    public void driveTrainSpeedTelemetry() {
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



    public void driveTrainEncoderTelemetry() {
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


}
