package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;


@Autonomous(name = "PerryPark 1 Tile", group="Autonomous")
//@Disabled
public class PerryPark1Tile extends LinearOpMode {

    private OLDRobot perry = new OLDRobot(this);


    @Override
    public void runOpMode() {

        perry.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            //perry.driveTrain.straightInches(TILE_LENGTH, 0.1);
            perry.driveTrain.strafeInches(TILE_LENGTH, 0.4);
            //while (!perry.sensors.isOverLine()) {}
            perry.driveTrain.brake();

        }

    }

}
