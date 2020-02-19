package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Fingers;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CustomTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.GamepadAdvanced;

import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_y;


@TeleOp(name = "Mechanum Strafing TeleOp")
public class GrucciMechanumTeleop extends OpMode {

    private final Robot robot = new Robot(this);

    private final double threshold = 0.0;


    private GamepadAdvanced gamepad1Advanced;
    private GamepadAdvanced gamepad2Advanced;


    @Override
    public void init() {
        robot.init(hardwareMap);

        gamepad1Advanced = new GamepadAdvanced(gamepad1);
        gamepad2Advanced = new GamepadAdvanced(gamepad2);
    }


    @Override
    public void init_loop() {
        CustomTelemetry.ok(telemetry);
        telemetry.update();
    }



    @Override
    public void start(){
        robot.outtake.submit();
        robot.fingers.pepeSMASHIn();
    }



    private void gamepad1Controls() {

        // Control over horizontal and vertical movement amounts with one joystick, and turn amount using the other.
        if (Math.abs(gamepad1.left_stick_y) >= threshold) {
            movement_y = -gamepad1.left_stick_y;
        } else {
            movement_y = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) >= threshold) {
            movement_x = gamepad1.left_stick_x;
        } else {
            movement_x = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) >= threshold) {
            movement_turn = gamepad1.right_stick_x*0.5; // 0.5 is turn reduction multiplier
        } else {
            movement_turn = 0;
        }

        // fixed straight/strafe problem with being flipped
        robot.driveTrain.applyMovement(movement_y, movement_x, movement_turn);


        // Intake toggling controls
        if (gamepad1Advanced.rightBumperOnce()) {
            if (robot.intake.getDirection() != Intake.Directions.SUCK) {

                robot.intake.suck();
                robot.outtake.openClaw();

            } else {
                robot.intake.rest();
            }
        }

        if (gamepad1Advanced.leftBumperOnce()) {
            if (robot.intake.getDirection() != Intake.Directions.BLOW) {
                robot.intake.blow();
            } else {
                robot.intake.rest();
            }
        }


        if (gamepad1Advanced.BOnce()) {
            if (robot.fingers.getPepeSMASHState() != Fingers.States.OUT) {
                robot.fingers.pepeSMASHOut();
            } else {
                robot.fingers.pepeSMASHIn();
            }
        }


        if (gamepad1Advanced.XOnce()) {

            if (robot.intake.buildSiteYanker.getPosition() != robot.intake.YANKED_POS) {
                robot.intake.yankBuildSite();
            } else {
                robot.intake.unYankBuildSite();
            }
        }

        gamepad1Advanced.update();
    }



    private void gamepad2Controls() {

        if (Math.abs(gamepad2.left_stick_y) >= threshold) {
            robot.outtake.setPulleySpeed(gamepad2.left_stick_y);
        } else {
            robot.outtake.stopPulley();
        }

        if (gamepad2Advanced.BOnce()) {

            robot.outtake.openClaw();
            robot.outtake.submit();
        }

        if (gamepad2Advanced.AOnce()) {

            robot.outtake.closeClaw();
            robot.outtake.zombieArms();
        }


        gamepad2Advanced.update();
    }



    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop() {

        gamepad1Controls();
        gamepad2Controls();


        telemetry.addData("Status", "Running");

        telemetry.addData("front left power", robot.driveTrain.leftFront.getPower());
        telemetry.addData("front right power", robot.driveTrain.rightFront.getPower());
        telemetry.addData("back left power", robot.driveTrain.leftRear.getPower());
        telemetry.addData("back right power", robot.driveTrain.rightRear.getPower());

        telemetry.addData("left intake power", robot.intake.leftIntake.getPower());
        telemetry.addData("right intake power", robot.intake.rightIntake.getPower());

        telemetry.addData("left pulley speed", robot.outtake.leftPulley.getPower());
        telemetry.addData("right pulley speed", robot.outtake.rightPulley.getPower());
        telemetry.addData("arm pos", robot.outtake.getElbow());
        telemetry.addData("claw pos", robot.outtake.getClawPos());
        telemetry.addData("left elbow pos", robot.outtake.leftElbow.getPower());
        telemetry.addData("right elbow pos", robot.outtake.rightElbow.getPower());

        CustomTelemetry.memeData(telemetry);

        //telemetry.addData("finger pos", robot.fingers.pepeSMASH.getPosition());

        telemetry.update();
    }


}