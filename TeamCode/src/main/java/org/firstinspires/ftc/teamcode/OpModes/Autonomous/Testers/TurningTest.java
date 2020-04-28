package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;

import static org.firstinspires.ftc.teamcode.Util.MiscUtil.pause;


@Autonomous(name = "TurningTest", group="Autonomous")
@Disabled
public class TurningTest extends LinearOpMode {

    private OLDRobot perry = new OLDRobot(this);


    @Override
    public void runOpMode() {



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            perry.driveTrain.turnToDeg(180, 0.6);
            pause(500);
            perry.driveTrain.turnToDeg(0, 0.5);
            pause(500);
            perry.driveTrain.turnToDeg(-90, 0.3);
            pause(500);
            perry.driveTrain.turnToDeg(-0, 0.45);
            pause(500);
            perry.driveTrain.turnToDeg(0, 0.4);
            pause(500);
            perry.driveTrain.turnToDeg(0, 0.4);

        }

        telemetry.addData("deg", perry.sensors.getWorldAngleDeg());
        telemetry.update();
        pause(10000);

    }

}
