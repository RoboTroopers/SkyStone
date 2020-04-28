package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.OLD.Fingers;
import org.firstinspires.ftc.teamcode.Hardware.OLD.Intake;
import org.firstinspires.ftc.teamcode.Hardware.OLD.OLDRobot;
import org.firstinspires.ftc.teamcode.Util.CustomTelemetry;
import org.firstinspires.ftc.teamcode.Util.GamepadAdvanced;

import static org.firstinspires.ftc.ppProject.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.ppProject.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.ppProject.RobotUtilities.MovementVars.movement_y;


@TeleOp(name = "Grucci Mechanum TeleOp")
public class GrucciMechanumTeleop extends OpMode {

    private OLDRobot robot = new OLDRobot(this);

    private final double threshold = 0.0;


    private GamepadAdvanced gamepad1Advanced;
    private GamepadAdvanced gamepad2Advanced;
    //private CustomTelemetry customTelemetry = new CustomTelemetry(robot, telemetry);

    private int liftTimer = 1000;
    private int depositTimer = 1000;

    private boolean tailOut = false;


    @Override
    public void init() {
        robot.init(hardwareMap);

        gamepad1Advanced = new GamepadAdvanced(gamepad1);
        gamepad2Advanced = new GamepadAdvanced(gamepad2);

        //customTelemetry.ok();
        telemetry.update();
    }


    @Override
    public void start(){

        robot.outtake.retractTailAuto();
        robot.fingers.pepeSMASHIn();
    }


    @Override
    public void loop() {
    //private void gamepad1Controls() {

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
    //}



    //private void gamepad2Controls() {

        if (Math.abs(gamepad2.left_stick_y) >= threshold) {
            robot.outtake.setPulleySpeed(gamepad2.left_stick_y);
        } else {
            robot.outtake.stopPulley();
        }


        if (Math.abs(gamepad2.right_stick_y) >= threshold) {
            robot.outtake.setTailSpeed(gamepad2.right_stick_y);
        } else {
            robot.outtake.stopTail();
        }


        // Use timer instead of sleep to not block other controls so they can work in parallel.
        if (gamepad2Advanced.leftBumperOnce()) {
            robot.outtake.closeClaw();
            liftTimer = 0;
            tailOut = true;
        }

        // Hold tail out for as long as possible
        if (liftTimer > 30 && tailOut) {
            robot.outtake.thrustTail();
        }

        liftTimer++;


        if (gamepad2Advanced.rightBumperOnce()) {
            robot.outtake.openClaw();
            depositTimer = 0;
            tailOut = false;
        }

        if (depositTimer == 36) {
            robot.outtake.retractTail();
        }

        depositTimer++;


        gamepad2Advanced.update();



    // Run until the end of the match (driver presses STOP)
    //@Override
    //public void loop() {

        //gamepad1Controls();
        //gamepad2Controls();


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
        telemetry.addData("left elbow pos", robot.outtake.leftTail.getPower());
        telemetry.addData("right elbow pos", robot.outtake.rightTail.getPower());

        //customTelemetry.memeData();

        //telemetry.addData("finger pos", robot.fingers.pepeSMASH.getPosition());

        if (this.getRuntime() < 9) {
            //customTelemetry.nerd();
        }


        telemetry.update();
    }


}