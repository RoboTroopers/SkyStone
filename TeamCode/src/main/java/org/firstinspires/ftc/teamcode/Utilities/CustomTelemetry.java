package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Utilities.Memes.chromosomes;
import static org.firstinspires.ftc.teamcode.Utilities.Memes.getRobotSpedness;
import static org.firstinspires.ftc.teamcode.Utilities.Memes.matb;


public class CustomTelemetry {



    public static void spacer(Telemetry telemetry) {
        telemetry.addData("","");
    }


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
        spacer(telemetry);
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
        spacer(telemetry);
    }


    public static void sensorData(Robot robot, Telemetry telemetry) {
        telemetry.addData("Angle", robot.sensors.getWorldAngleDeg());
        telemetry.addData("Is over Line", robot.sensors.isOverLine());
        telemetry.addData("Red side", robot.sensors.isRedSide());
        telemetry.addData("Distance from stone", robot.sensors.getStoneDistance());
        telemetry.addData("Holding stone", robot.sensors.holdingStone());
        telemetry.addData("Intaking stone", robot.sensors.intakingStone());
        telemetry.addData("Pulley height", robot.outtake.getHeight());
        spacer(telemetry);
    }


    public static void memeData(Telemetry telemetry) {
        telemetry.addData("Robot spedness", getRobotSpedness());
        telemetry.addData("Aqui esta un muy dank meme para tus madre", matb);
        telemetry.addData("May I make a small request", chromosomes);
        spacer(telemetry);

    }



    public static void ok(Telemetry telemetry) {
        telemetry.addData("",
          "         " +
                "     _   " +
                "   _| |\n" +
                " _| | |\n" +
                "| | | |\n" +
                "| | | | __\n" +
                "| | | |/  \\\n" +
                "|       /\\ \\\n" +
                "|      /  \\/\n" +
                "|      \\  /\\\n" +
                "|       \\/ /\n" +
                " \\        /\n" +
                "  |     /\n" +
                "  |    |"
        );
    }



}
