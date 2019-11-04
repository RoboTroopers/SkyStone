package org.firstinspires.ftc.teamcode.Robot;

import android.provider.Settings;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Odometry.GlobalPositionTracker;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.MathFunctions.inchesToTicks;



public class Robot {
    
    // Motors and servos
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    public Servo pinger;

    public DcMotor leftIntake;
    public DcMotor rightIntake;
    
    // Sensors
    public BNO055IMU imu;
    public DcMotor xEncoder;
    public DcMotor yEncoder;
    
    
    public double getXEncoder() {
        odometry.updatePos();
        return xEncoder.getCurrentPosition();
    }

    public double getYEncoder() {
        odometry.updatePos();
        return yEncoder.getCurrentPosition();
    }

    public void resetEncoders() {
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    

    public GlobalPositionTracker odometry;

    public double worldXPosition;
    public double worldYPosition;
    public double worldAngle_rad;


    public double getXPos(){
        return worldXPosition;
    }
    
    public double getYPos() {
        return worldYPosition;
    }
    

    public double movement_x;
    public double movement_y;
    public double movement_turn;



    public Robot(double xInches, double yInches, double degrees) {
        
        odometry.setPosition(this, xInches, yInches, toRadians(degrees));
        
    }
    
    public void initHardware (HardwareMap aHwMap) {
        
        leftFront = aHwMap.get(DcMotor.class, "leftFront");
        rightFront = aHwMap.get(DcMotor.class, "rightFront");
        leftRear = aHwMap.get(DcMotor.class, "leftRear");
        rightRear = aHwMap.get(DcMotor.class, "rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        pinger = aHwMap.get(Servo.class, "pinger");
        
        leftIntake = aHwMap.get(DcMotor.class, "leftIntake");
        rightIntake = aHwMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        
        imu = aHwMap.get(BNO055IMU.class, "imu");
        xEncoder = aHwMap.get(DcMotor.class, "xEncoder");
        yEncoder = aHwMap.get(DcMotor.class, "yEncoder");
        resetEncoders();
        
    }
    

    // Stations the robot in current position
    public void brake() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    
    
    // Moves all motors at same power
    public void forward(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);

    }
    
    
    // Moves the left and right side motors separate speeds
    public void steer(double leftSpeed, double rightSpeed) {
        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);

    }


    // Moves the robot sideways without turning
    public void strafe(double speed){
        // Positive speed strafes right, negative speed strafes left.
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftRear.setPower(-speed);
        rightRear.setPower(speed);

    }
    
    
    public void turnToDegrees(double degrees, double maxSpeed) {
        
        // positive degrees == right; negative degrees == left
        double turnDegrees = degrees;
        
        double targetRange = 0.5; // Distance from the desired angle that is allowed
        double error = turnDegrees; // Distance from the desired range
        double progress; // Degrees the robot has turned already
        double minSpeed = 0.01; // Lowest speed the motors will go; Turning is generally more precise and accurate when lower.
        double speed; // Actual speed value of the motors
        
        while (Math.abs(error) > targetRange) {
            progress = turnDegrees-Math.toDegrees(worldAngle_rad);
            error = progress-turnDegrees;
            
            speed = ((1-(progress/turnDegrees))*maxSpeed)+minSpeed; // Speed starts at maximum and approaches minimum as the gyro value approaches the desired angle. It deccelerates for precision and accuray.
            speed = Range.clip(speed, minSpeed, maxSpeed);
            
            if (turnDegrees < 0)
                steer(-speed, speed);
            else
                steer(speed, -speed);
        }
        
        brake();
    }


    public void goToPosition(double xInches, double yInches, double movementSpeed, double preferredAngle, double turnSpeed) {
        double accuracyRange = inchesToTicks(0.5);
        double x = inchesToTicks(xInches);
        double y = inchesToTicks(yInches);
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);



        while (!(distanceToTarget > -accuracyRange && distanceToTarget < accuracyRange)) {
            distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (worldAngle_rad - toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle;
            movement_turn = Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            if(Math.abs(distanceToTarget) < 3) {
                movement_turn = 0;
            }

            moveXYTurn(movement_x, movement_y, movement_turn);

        }
    }
    
    
    public void moveXYTurn(double x, double y, double turn) {
        //translates robot position on field by x and y, rotates by turn
        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double lf_power_raw = y - turn + (x*1.5);
        double rf_power_raw = -y - turn + (x*1.5);
        double lr_power_raw = y - turn - (x*1.5);
        double rr_power_raw = -y - turn + (x*1.5);
        
        // Find greatest power
        double maxRawPower = Math.max(Math.max(lf_power_raw, rf_power_raw), Math.max(lr_power_raw, rr_power_raw));
        
        double scaleDownFactor = 1.0;
        if (maxRawPower > 1.0) {
            // Reciprocal of maxRawPower so that when multiplied by factor, maxPower == 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }
        
        // All motor speeds scaled down (if maxRawPower > 1) but vector is preserved.
        lr_power_raw *= scaleDownFactor;
        rf_power_raw *= scaleDownFactor;
        lr_power_raw *= scaleDownFactor;
        rr_power_raw *= scaleDownFactor;
        
        // Changes motor powers only if they have changed
        if (leftFront.getPower() != lf_power_raw)
            leftFront.setPower(lf_power_raw);
        if (rightFront.getPower() != rf_power_raw)
            rightFront.setPower(rf_power_raw);
        if (leftRear.getPower() != lr_power_raw)
            leftRear.setPower(lr_power_raw);
        if (rightRear.getPower() != rr_power_raw)
            rightRear.setPower(rr_power_raw);
        
    }
    
    
    
    // Extends pinger to its maximum length
    public void pingerOut() {
        pinger.setPosition(0.25);
        
    }
    
    
    // Retracts pinger into the robot
    public void pingerIn() {
        pinger.setPosition(0);
        
    }
    
    
    // Retracts pinger into the robot
    public void startSucc(double speed) {
        leftIntake.setPower(speed);
        
    }
    
    
    // Retracts pinger into the robot
    public void stopSucc() {
        leftIntake.setPower(0);
        
    }
    
    
}