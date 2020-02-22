package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;


@Autonomous(name = "PerryPark", group="Autonomous")
@Disabled
public class PerryPark extends LinearOpMode {

    private Robot perry = new Robot(this);


    @Override
    public void runOpMode() {
        perry.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();


        if (opModeIsActive()) {

            //perry.driveTrain.straight(0.4);
            //sleep(1000);
            //sleep(28000);
            perry.driveTrain.straightInches(TILE_LENGTH,0.4);
            perry.driveTrain.brake();
        }

    }

}
