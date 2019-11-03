package org.firstinspires.ftc.teamcode.AutonomousOpmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OdometryThread;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "sexDrive", group = "")
public class TestDrive extends LinearOpMode {

    Robot robot = new Robot(0,0,0);

    @Override
    public void runOpMode() {

        Thread t1 = new Thread(new OdometryThread(robot));
        robot.initHardware(hardwareMap);

        waitForStart();
        t1.start();
        robot.goToPosition(0, 24, 0.5, 0, 0.25);
        telemetry.addData("programX", robot.worldXPosition);
        telemetry.addData("programY", robot.worldYPosition);
        telemetry.addData("currXLeft", robot.leftFront.getCurrentPosition());
        telemetry.addData("currXRight", robot.rightFront.getCurrentPosition());
        
    }
}
