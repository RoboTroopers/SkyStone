package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;


@Autonomous(name = "TurningTest", group="Autonomous")
@Disabled
public class TurningTest extends LinearOpMode {

    private Robot perry = new Robot(this, OpModeTypes.AUTO);


    @Override
    public void runOpMode() {

        perry.initHardware(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            perry.driveTrain.turnToDeg(180, 0.6);
            pause(500);
            perry.driveTrain.turnToDeg(0, 0.7);
            pause(500);
            perry.driveTrain.turnToDeg(-90, 0.3);
            pause(500);
            perry.driveTrain.turnToDeg(90, 0.5);

        }

        telemetry.addData("deg", perry.sensors.getWorldAngleDeg());
        telemetry.update();
        pause(10000);

    }

}
