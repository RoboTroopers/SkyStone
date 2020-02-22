package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Autonomous(name = "PerryPark Line Sensor TEST", group="Autonomous")
//@Disabled
public class PerryParkLineSensor extends LinearOpMode {

    private Robot perry = new Robot(this);


    @Override
    public void runOpMode() {

        perry.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()&& !opModeIsActive()) {
            //perry.driveTrain.straightInches(TILE_LENGTH, 0.75);
            perry.driveTrain.straight(0.25);

            while (!perry.sensors.isOverLine()) {
            }
            perry.driveTrain.brake();

        }

    }

}
