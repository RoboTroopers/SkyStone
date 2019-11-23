package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.movement_y;


@TeleOp(name = "Mechanum Strafing TeleOp")
public class MechanumStrafingTeleOp extends OpMode {

    private Robot robot = new Robot();

    private double threshold = 0.0;

    boolean armOut = false;
    boolean clawOpen = false;

    boolean returnArm = false;
    int clawReleasedCooldown = 1000;


    @Override
    public void init() {
        robot.initHardware(hardwareMap);
        //Thread odometryThread = new Thread(new OdometryThread(robot));
        //odometryThread.start();

    }


    // Run until the end of the match (driver presses STOP)
    @Override
    public void loop() {


        // Control over horizontal and vertical movement amounts with one joystick, and turn amount using the other.
        if (Math.abs(gamepad1.left_stick_x) >= threshold) {
            movement_x = gamepad1.left_stick_x;

        } else {
            movement_x = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) >= threshold) {
            movement_y = -gamepad1.left_stick_y;

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


        if (gamepad2.right_bumper) {

            if (!clawOpen) {
                robot.outtake.openClaw();
                clawOpen = true;

             } else {
                robot.outtake.closeClaw();
                clawOpen = false;

                if (armOut) { // Tell arm to return to normal position after stone is released
                    returnArm = true;
                    clawReleasedCooldown = 0;
                }
            }
        }


        if (gamepad2.left_bumper) {

            if (!armOut) {
                robot.outtake.armOut();
                armOut = true;

            } else {
                robot.outtake.armIn();
                armOut = false;
            }
        }


        if (returnArm && armOut && !clawOpen) {
            clawReleasedCooldown += 1;

            if (clawReleasedCooldown > 2000) {
                robot.outtake.armIn();
                armOut = false;
                returnArm = false;
            }
        } else {
            returnArm = false;
        }


        telemetry.addData("Status", "Running");

        telemetry.addData("front left power", robot.driveTrain.leftFront.getPower());
        telemetry.addData("front right power", robot.driveTrain.rightFront.getPower());
        telemetry.addData("back left power", robot.driveTrain.leftRear.getPower());
        telemetry.addData("back right power", robot.driveTrain.rightRear.getPower());

        telemetry.addData("left intake power", robot.intake.leftIntake.getPower());
        telemetry.addData("right intake power", robot.intake.rightIntake.getPower());

        telemetry.addData("arm pos", robot.outtake.getArmPos());
        telemetry.addData("wrist pos", -robot.outtake.getWristPos());
        telemetry.addData("claw pos", robot.outtake.getClawPos());
        telemetry.update();

    }
}

