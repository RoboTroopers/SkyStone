package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.OdometrySystem.OdometryThread;

@Autonomous(name = "TestDrive", group = "Autonomous")
public class TestDrive extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() {
        //robot.odometry.ssetPosition(0, 0, 0);
        robot.initHardware(hardwareMap);
        Thread t1 = new Thread(new OdometryThread(robot));

        waitForStart();
        t1.start();
        //robot.advancedMovement.myGoToPosition(0, 24, 0.5, 0, 0.25);
        //telemetry.addData("programX", robot.odometry.worldXPosition);
        //telemetry.addData("programY", robot.odometry.worldYPosition);
        //telemetry.addData("programY", robot.odometry.worldYPosition);
        //telemetry.addData("currXLeft", robot.sensing.horizontalEncoder.getCurrentPosition());
        telemetry.addData("currXRight", robot.sensing.horizontalEncoder.getCurrentPosition());
        
    }
}
