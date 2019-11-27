package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Finger;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.GamepadAdvanced;

import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_y;


@TeleOp(name = "Mechanum Strafing TeleOp")
public class MechanumStrafingTeleOp extends OpMode {

    private Robot robot = new Robot();

    private double threshold = 0.0;

    boolean armOut = false;

    int depositStoneTimer = 1000; // arbitrary
    int resetArmTimer = 1000;

    GamepadAdvanced gamepadAdvanced1;
    GamepadAdvanced gamepadAdvanced2;


    @Override
    public void init() {
        robot.initHardware(hardwareMap);

        gamepadAdvanced1 = new GamepadAdvanced(gamepad1);
        gamepadAdvanced2 = new GamepadAdvanced(gamepad2);
    }


    @Override
    public void start(){

        robot.outtake.armMid();

    }


    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop() {


        // Control over horizontal and vertical movement amounts with one joystick, and turn amount using the other.
        if (Math.abs(gamepad1.left_stick_y) >= threshold) {
            movement_x = gamepad1.left_stick_y;

        } else {
            movement_x = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) >= threshold) {
            movement_y = -gamepad1.left_stick_x;

        } else {
            movement_y = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) >= threshold) {// || Math.abs(gamepad1.right_stick_y) > threshold) {
            movement_turn = -gamepad1.right_stick_x;

        } else {
            movement_turn = 0;
        }

        robot.driveTrain.applyMovement(movement_x, movement_y, movement_turn);



        // Intake toggling controls
        if (gamepad1.right_bumper) {
            if (robot.intake.getDirection() != Intake.Directions.SUCK) {

                robot.intake.suck();
            } else {
                robot.intake.rest();
            }
        }

        if (gamepad1.left_bumper) {
            if (robot.intake.getDirection() != Intake.Directions.BLOW) {

                robot.intake.blow();
            } else {
                robot.intake.rest();
            }
        }


        if (Math.abs(gamepad2.left_stick_y) >= threshold) {

            robot.outtake.setPulleySpeed(gamepad2.left_stick_y);
        } else {
            robot.outtake.stopPulley();
        }


        if (gamepadAdvanced2.rightBumperOnce()) {

            //if (robot.outtake.claw.getPosition() == robot.outtake.CLAW_CLOSED_POS) {
                robot.outtake.openClaw();

             /*} else {
                robot.outtake.closeClaw();
            }*/
        }


        if (gamepadAdvanced2.leftBumperOnce()) {

            if (robot.outtake.arm.getPosition() != robot.outtake.ARM_OUT_POS) {
                depositStoneTimer = 0;
                resetArmTimer = 1000;

            } else {
                resetArmTimer = 0;
                depositStoneTimer = 1000;

            }
        }


        if (depositStoneTimer <= 110) {

            if (depositStoneTimer == 2) {
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
        
        
        if (gamepadAdvanced1.AOnce()) {
            if (robot.pepeSMASH.getState() == Finger.States.DOWN) {
                robot.pepeSMASH.up();
            } else {
                robot.pepeSMASH.down();
            }
                
        }
        


        gamepadAdvanced1.update();
        gamepadAdvanced2.update();


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

        telemetry.addData("deposit stone timer", depositStoneTimer);
        telemetry.addData("reset arm timer", resetArmTimer);
        telemetry.update();

    }
}

