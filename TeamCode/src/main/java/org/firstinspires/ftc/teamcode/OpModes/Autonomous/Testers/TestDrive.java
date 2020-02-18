package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "TestDrive", group = "Autonomous")
@Disabled
public class TestDrive extends LinearOpMode {

    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        //robot.odometry.setPosition(0, 0, 0);
        robot.init(hardwareMap);
        //Thread t1 = new Thread(new OdometryThread(robot));
        
        waitForStart();
        //t1.start();
        //robot.advancedMovement.myGoToPosition(0, 24, 0.5, 0, 0.25);
        //telemetry.addData("programX", robot.odometry.worldXPosition);
        //telemetry.addData("programY", robot.odometry.worldYPosition);
        //telemetry.addData("programY", robot.odometry.worldYPosition);
        //telemetry.addData("currXLeft", robot.sensors.horizontalEncoder.getCurrentPosition());
        //telemetry.addData("currXRight", robot.sensors.horizontalEncoder.getCurrentPosition());
        
    }
}
