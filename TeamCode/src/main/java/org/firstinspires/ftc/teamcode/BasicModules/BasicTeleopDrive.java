package org.firstinspires.ftc.teamcode.BasicModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "BasicTeleopDrive")
public class BasicTeleopDrive extends OpMode {
    
    /*
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    */
    Robot robot = new Robot();
    
    
    
    @Override
    public void init() {
        /*
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        */
        
        robot.initHardware(hardwareMap);
    }
        

        // run until the end of the match (driver presses STOP)
    public void loop() {
        //Drive motor control

        double threshold = 0.157;

        /*rightFront.setPower((gamepad1.left_stick_y));
        leftFront.setPower((gamepad1.left_stick_y));
        leftRear.setPower((gamepad1.left_stick_y));
        rightRear.setPower((gamepad1.left_stick_y));

         */
        
        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)  {
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

        
        if(Math.abs(gamepad1.right_stick_x) > threshold) {
            robot.rightFront.setPower((gamepad1.right_stick_x));
            robot.leftFront.setPower((-gamepad1.right_stick_x));
            robot.leftRear.setPower((-gamepad1.right_stick_x));
            robot.rightRear.setPower((gamepad1.right_stick_x));
        }
        
        
        while(gamepad1.right_trigger > 0.5) {
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
        }


        telemetry.addData("Status", "Running");
        telemetry.addData("front left power", robot.leftFront.getPower());
        telemetry.addData("front right power", robot.rightFront.getPower());
        telemetry.addData("back left power", robot.leftRear.getPower());
        telemetry.addData("back right power", robot.rightRear.getPower());
        telemetry.addData("left intake power", robot.leftIntake.getPower());
        telemetry.addData("right intake power", robot.rightIntake.getPower());
        telemetry.update();

    }


}

