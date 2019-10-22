package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoboConfig;


/**
 *
 * Created by Joshua Herer on 8/28/19
 *
 * Task:
 * - Go straight for 3 seconds
 * - turn right 3 times
 * - go straight 4 seconds
 * - turn left
 * - go straight 1.5 seconds
 * - turn 360
 *
 *
 * Drive: Mechanum Drive
 *
 */


@Autonomous
public class ChallengeOpMode1 extends LinearOpMode {

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor rearLeftMotor;


    @Override
    public void runOpMode() {

        RoboConfig robot = new RoboConfig();
        HardwareMap hardwareMap = new HardwareMap(null);
        robot.initHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.steer(0.8, 0.8);
            sleep(3000);
            robot.steer(0.5, -0.5);
            sleep(6000);
            robot.brake();
            robot.steer(0.8, 0.8);
            sleep(4000);
            robot.steer(-0.5, 0.5);
            sleep(500);
            robot.brake();
            robot.steer(0.8, 0.8);
            sleep(1500);
            robot.steer(0.5, -0.5);
            sleep(2000);
            robot.brake();

        }

    }
    
}
