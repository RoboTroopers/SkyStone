package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "BackwardsTest", group="Autonomous")
@Disabled
public class BackwardsTest extends LinearOpMode {

    private Robot perry = new Robot(this);


    @Override
    public void runOpMode() {



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            perry.driveTrain.straightInches(20, 0.15);
            perry.driveTrain.backwardInches(20, 0.15);
            perry.driveTrain.brake();

        }

    }

}
