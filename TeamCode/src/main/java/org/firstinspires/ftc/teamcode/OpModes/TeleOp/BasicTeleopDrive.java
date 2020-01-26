package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;


@TeleOp(name = "BasicTeleopDrive")
@Disabled
public class BasicTeleopDrive extends OpMode {

    Robot robot = new Robot(this, OpModeTypes.TELEOP);



    @Override
    public void init() {
        robot.initHardware(hardwareMap);
    }


    // run until the end of the match (driver presses STOP)
    public void loop() {
        //Drive motor control

        double threshold = 0.157;


        if (Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)  {
            robot.driveTrain.rightFront.setPower(((gamepad1.left_stick_y) + (gamepad1.left_stick_x)));
            robot.driveTrain.leftRear.setPower(((gamepad1.left_stick_y) + (gamepad1.left_stick_x)));
            robot.driveTrain.leftFront.setPower(((gamepad1.left_stick_y) - (gamepad1.left_stick_x)));
            robot.driveTrain.rightRear.setPower(((gamepad1.left_stick_y) - (gamepad1.left_stick_x)));
        } else {
            robot.driveTrain.leftFront.setPower(0);
            robot.driveTrain.rightFront.setPower(0);
            robot.driveTrain.leftRear.setPower(0);
            robot.driveTrain.rightRear.setPower(0);
        }


        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            robot.driveTrain.rightFront.setPower((gamepad1.right_stick_x));
            robot.driveTrain.leftFront.setPower((-gamepad1.right_stick_x));
            robot.driveTrain.leftRear.setPower((-gamepad1.right_stick_x));
            robot.driveTrain.rightRear.setPower((gamepad1.right_stick_x));
        }


        if (gamepad1.right_trigger > 0.5) {
            robot.intake.blow();

        } else if (gamepad1.left_trigger > 0.5) {

            robot.intake.suck();

        } else {
            robot.intake.rest();
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("front left power", robot.driveTrain.leftFront.getPower());
        telemetry.addData("front right power", robot.driveTrain.rightFront.getPower());
        telemetry.addData("back left power", robot.driveTrain.leftRear.getPower());
        telemetry.addData("back right power", robot.driveTrain.rightRear.getPower());
        //telemetry.addData("left intake power", robot.intake.leftIntake.getPower());
        telemetry.addData("right intake power", robot.intake.rightIntake.getPower());
        telemetry.update();
    }
}

