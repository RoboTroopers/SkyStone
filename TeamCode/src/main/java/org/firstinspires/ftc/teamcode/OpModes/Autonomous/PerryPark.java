package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Globals.FieldConstants;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;


@Autonomous(name = "PerryPark", group="Autonomous")
//@Disabled
public class PerryPark extends LinearOpMode {

    private Robot perry = new Robot(this, OpModeTypes.AUTO);


    @Override
    public void runOpMode() {

        perry.initHardware(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            perry.driveTrain.straightInches(TILE_LENGTH, 0.75);
            //while (!perry.sensors.overLine()) {}
            perry.driveTrain.brake();

        }

    }

}
