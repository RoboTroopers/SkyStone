package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoboConfig;


@TeleOp(name = "BasicMechanumTeleOp")
public class BasicMechanumTeleOp extends LinearOpMode {
    
    
    @Override
    public void runOpMode() {

        RoboConfig robot = new RoboConfig();
        HardwareMap hardwareMap = new HardwareMap(null);
        robot.initHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {
            //Drive motor control

            double threshold = 0.157;

            /*rightFront.setPower((gamepad1.left_stick_y));
            leftFront.setPower((gamepad1.left_stick_y));
            leftRear.setPower((gamepad1.left_stick_y));
            rightRear.setPower((gamepad1.left_stick_y));
             */

            // Steering left, right, and straight
            if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold) {
                
                robot.rightFront.setPower(((gamepad1.left_stick_y) + (gamepad1.left_stick_x)));
                robot.leftRear.setPower(((gamepad1.left_stick_y) + (gamepad1.left_stick_x)));
                robot.leftFront.setPower(((gamepad1.left_stick_y) - (gamepad1.left_stick_x)));
                robot.rightRear.setPower(((gamepad1.left_stick_y) - (gamepad1.left_stick_x)));
                
            } else {
                
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
            }
            
            // Strafing 
            if(Math.abs(gamepad1.right_stick_x) > threshold) {
                robot.rightFront.setPower((gamepad1.right_stick_x));
                robot.leftFront.setPower((-gamepad1.right_stick_x));
                robot.leftRear.setPower((-gamepad1.right_stick_x));
                robot.rightRear.setPower((gamepad1.right_stick_x));
                
            }
            
            if (gamepad2.right_trigger == 1) {
                
                robot.leftIntake.setPower(1);
                robot.rightIntake.setPower(1);
                
            } else {
                
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            }
            
            telemetry.addData("Status", "Running");
            telemetry.addData("front left power", robot.leftFront.getPower());
            telemetry.addData("front right power", robot.rightFront.getPower());
            telemetry.addData("back left power", robot.leftRear.getPower());
            telemetry.addData("back right power", robot.rightRear.getPower());
            telemetry.update();
            
        }
    
    
    }
}
