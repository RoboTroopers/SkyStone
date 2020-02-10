package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


@Autonomous(name = "Yank it baby", group="Autonomous")
//@Disabled
public class YankIt extends LinearOpMode {

    private Robot robot = new Robot(this, OpModeTypes.AUTO);


    @Override
    public void runOpMode() {

        robot.initHardware(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        robot.intake.unYankStone();

        waitForStart();

        if (opModeIsActive()) {

            robot.intake.yankStone();
            sleep(3000);
            robot.intake.unYankStone();

        }
    }

}