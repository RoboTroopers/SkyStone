package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;


@Autonomous(name = "PerryPark", group="Autonomous")
//@Disabled
public class PerryParkWait extends LinearOpMode {
    
    private Robot perry = new Robot();


    @Override
    public void runOpMode() {
        
        perry.initHardware(hardwareMap);
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        
        waitForStart();


        if (opModeIsActive()) {
            /*
            int distance = (int)inchesToTicks(12);
            perry.driveTrain.setTargetPos(distance, distance, distance, distance);
            perry.driveTrain.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

            perry.driveTrain.straight(0.5);
            while (!perry.driveTrain.anyMotorsBusy()) {
                telemetry.addData("Inches", perry.driveTrain.getEncoderAvgInches());
                pause(10);
            }

             */

            pause(28000);
            perry.driveTrain.straight(7);
            pause(1000);
            perry.driveTrain.brake();
        }

    }

}
