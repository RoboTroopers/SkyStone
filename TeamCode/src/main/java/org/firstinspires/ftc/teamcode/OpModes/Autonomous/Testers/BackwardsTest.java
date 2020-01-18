package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


@Autonomous(name = "BackwardsTest", group="Autonomous")
//@Disabled
public class BackwardsTest extends LinearOpMode {

    private Robot perry = new Robot(this, OpModeTypes.AUTO);


    @Override
    public void runOpMode() {

        perry.initHardware(hardwareMap);

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
