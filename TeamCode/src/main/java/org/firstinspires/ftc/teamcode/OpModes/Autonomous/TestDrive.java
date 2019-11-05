package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.OdometryThread;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "TestDrive", group = "Autonomous")
public class TestDrive extends LinearOpMode {

    Robot robot = new Robot(0,0,0);

    @Override
    public void runOpMode() {

        Thread t1 = new Thread(new OdometryThread(robot));
        robot.initHardware(hardwareMap);

        waitForStart();
        t1.start();
        robot.advancedMovement.goToPosition(0, 24, 0.5, 0, 0.25);
        telemetry.addData("programX", robot.odometry.worldXPosition);
        telemetry.addData("programY", robot.odometry.worldYPosition);
        telemetry.addData("programY", robot.odometry.worldYPosition);
        telemetry.addData("currXLeft", robot.sensors.horizontalEncoder.getCurrentPosition());
        telemetry.addData("currXRight", robot.sensors.horizontalEncoder.getCurrentPosition());
        
    }
}
