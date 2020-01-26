package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


@Autonomous(name = "PerryPark Line Sensor TEST", group="Autonomous")
//@Disabled
public class PerryParkLineSensor extends LinearOpMode {

    private Robot perry = new Robot(this, OpModeTypes.AUTO);


    @Override
    public void runOpMode() {

        perry.initHardware(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //perry.driveTrain.straightInches(TILE_LENGTH, 0.75);
            perry.driveTrain.straight(0.05);

            while (!perry.sensors.isOverLine()) {
            }
            perry.driveTrain.brake();

        }

    }

}
