package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;


@Autonomous(name = "Yank it baby", group="Autonomous")
@Disabled
public class YankIt extends LinearOpMode {

    private OLDRobot robot = new OLDRobot(this);


    @Override
    public void runOpMode() {



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        robot.intake.unYankBuildSite();

        waitForStart();

        if (opModeIsActive()) {

            robot.intake.yankBuildSite();
            sleep(3000);
            robot.intake.unYankBuildSite();

        }
    }

}