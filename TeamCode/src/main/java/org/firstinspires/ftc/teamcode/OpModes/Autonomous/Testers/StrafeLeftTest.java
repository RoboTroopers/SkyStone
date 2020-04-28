package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;


@Autonomous(name = "Strafe Left Test", group="Autonomous")
@Disabled
public class StrafeLeftTest extends LinearOpMode {

    private OLDRobot perry = new OLDRobot(this);


    @Override
    public void runOpMode() {



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            perry.driveTrain.strafeInches(-3*TILE_LENGTH, 0.1);

        }

    }

}
