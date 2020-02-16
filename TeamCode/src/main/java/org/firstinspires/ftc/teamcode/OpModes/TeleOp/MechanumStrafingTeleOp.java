package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Fingers;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.GamepadAdvanced;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_y;


@TeleOp(name = "Mechanum Strafing TeleOp")
public class MechanumStrafingTeleOp extends OpMode {

    private Robot robot = new Robot(this, OpModeTypes.TELEOP);

    private double threshold = 0.0;

    boolean armOut = false;

    private int depositStoneTimer = 1000; // arbitrary
    private int resetArmTimer = 1000;

    private GamepadAdvanced gamepad1Advanced;
    private GamepadAdvanced gamepad2Advanced;


    @Override
    public void init() {
        robot.initHardware(hardwareMap);

        gamepad1Advanced = new GamepadAdvanced(gamepad1);
        gamepad2Advanced = new GamepadAdvanced(gamepad2);
    }


    @Override
    public void start(){

        robot.outtake.armMid();
        robot.fingers.pepeSMASHIn();

    }


    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop() {


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

        if (Math.abs(gamepad1.right_stick_x) >= threshold) {// || Math.abs(gamepad1.right_stick_y) > threshold) {
            movement_turn = -gamepad1.right_stick_x*0.5;

        } else {
            movement_turn = 0;
        }


        // fixed straight/strafe problem with being flipped
        robot.driveTrain.applyMovement(movement_y, movement_x, movement_turn);


        // Intake toggling controls
        if (gamepad1Advanced.rightBumperOnce()) {
            if (robot.intake.getDirection() != Intake.Directions.SUCK) {

                robot.intake.suck();
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
                //robot.fingers.pepeSMASH.setPosition(robot.fingers.PEPESMASH_OUT_POS);
            } else {
                robot.fingers.pepeSMASHIn();
                //robot.fingers.pepeSMASH.setPosition(robot.fingers.pepeSMASH.PEPESMASH_IN_POS);
            }
        }


        if (Math.abs(gamepad2.left_stick_y) >= threshold) {

            robot.outtake.setPulleySpeed(gamepad2.left_stick_y);
        } else {
            robot.outtake.stopPulley();
        }


        if (gamepad2Advanced.rightBumperOnce()) {

            //if (robot.outtake.claw.getPosition() == robot.outtake.CLAW_CLOSED_POS) {
            robot.outtake.openClaw();

             //} else {robot.outtake.closeClaw();}
        }


        if (gamepad2Advanced.leftBumperOnce()) {

            if (robot.outtake.arm.getPosition() != robot.outtake.ARM_OUT_POS) {
                depositStoneTimer = 0; // Starts timer to pick up the stone and place it on the ground
                resetArmTimer = 1000; // Arbitrary value above timer threshold

            } else {
                resetArmTimer = 0;
                depositStoneTimer = 1000;

            }
        }


        if (gamepad1Advanced.XOnce()) {

            if (robot.intake.stoneYanker.getPosition() != robot.intake.YANKED_POS) {
                robot.intake.yankStone();
            } else {
                robot.intake.unYankStone();
            }
        }


        if (robot.sensors.holdingStone()) {
            robot.fingers.pepeSMASHIn();
        }

        /*
        if (robot.sensors.stoneFullyIn()) {
            depositStoneTimer = 0;
            resetArmTimer = 1000;
        }*/

        if (depositStoneTimer <= 110) {

            if (depositStoneTimer == 2) {
                robot.fingers.pepeSMASH.setPosition(robot.fingers.PEPESMASH_IN_POS);
                robot.outtake.armMid();
                robot.outtake.openClaw();

            } else if (depositStoneTimer == 45) {
                robot.outtake.armIn();

            } else if (depositStoneTimer == 70) {
                robot.outtake.closeClaw();

            } else if (depositStoneTimer == 110) {
                robot.outtake.armOut();
            }

            depositStoneTimer += 1;
        }


        if (resetArmTimer <= 40) {
            if (resetArmTimer == 2) {
                ///robot.outtake.openClaw();

            } else if (resetArmTimer == 40) {
                robot.outtake.armMid();
            }
            resetArmTimer += 1;
        }


        if (gamepad1Advanced.AOnce()) {
            if (robot.fingers.getPepeSMASHState() != Fingers.States.OUT) {
                robot.fingers.pepeSMASHOut();
            } else {
                robot.fingers.pepeSMASHIn();
            }

        }



        gamepad1Advanced.update();
        gamepad2Advanced.update();


        telemetry.addData("Status", "Running");

        telemetry.addData("front left power", robot.driveTrain.leftFront.getPower());
        telemetry.addData("front right power", robot.driveTrain.rightFront.getPower());
        telemetry.addData("back left power", robot.driveTrain.leftRear.getPower());
        telemetry.addData("back right power", robot.driveTrain.rightRear.getPower());

        //telemetry.addData("left intake power", robot.intake.leftIntake.getPower());
        //telemetry.addData("right intake power", robot.intake.rightIntake.getPower());

        telemetry.addData("arm pos", robot.outtake.getArmPos());
        telemetry.addData("wrist pos", -robot.outtake.getWristPos());
        telemetry.addData("claw pos", robot.outtake.getClawPos());

        telemetry.addData("finger pos", robot.fingers.pepeSMASH.getPosition());

        telemetry.addData("deposit stone timer", depositStoneTimer);
        telemetry.addData("reset arm timer", resetArmTimer);
        telemetry.update();

    }
}

