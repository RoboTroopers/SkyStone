package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.teamcode.MathFunctions.inchesToTicks;
import static org.firstinspires.ftc.teamcode.MathFunctions.restrictToRange;



public class Robot {
    
    // Motors and servos
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    public Servo pinger = null;

    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    
    // Sensors
    public BNO055IMU imu = null;
    public DcMotor xEncoder = null;
    public DcMotor yEncoder = null;
    
    
    // Odometry variables
    public double xSpeed = 0;
    public double ySpeed = 0;
    public double turnSpeed = 0;
    
    public double movement_x;
    public double movement_y;
    public double movement_turn;
    
    
    public double worldXPosition;
    public double worldYPosition;
    public double worldAngle_rad;
    

    public double getXPos(){
        return worldXPosition;
    }
    public double getYPos(){ return worldYPosition; }
    
    public double xEncoderLast;
    public double yEncoderLast;
    
    public double getXEncoder() { 
        updatePos();
        return xEncoder.getCurrentPosition();
    }
    public double getYEncoder() {
        updatePos();
        return yEncoder.getCurrentPosition();
    }
    
    
    public void resetEncoders() {
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    
    public void updatePos() {
        worldAngle_rad = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;

        double xEncoderChange = getXEncoder() - xEncoderLast;
        double yEncoderChange = getYEncoder() - yEncoderLast;

        worldXPosition += yEncoderChange * Math.cos(worldAngle_rad);
        worldYPosition += yEncoderChange * Math.sin(worldAngle_rad);

        xEncoderLast = getXEncoder();
        yEncoderLast = getYEncoder();

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
        rightIntake = aHwMap.get(DcMotor.class, "righIntake");
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

    // Moves the robot sideways without turning
    public void strafe(double leftFrontSpeed, double rightFrontSpeed, double leftRearSpeed, double rightRearSpeed){
        // Having 2 motors on one side which turn towards each other strafes that direction
        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftRear.setPower(leftRearSpeed);
        rightRear.setPower(rightRearSpeed);

    }
    

    void turnToDegrees(float degrees, double maxSpeed) {

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
            speed = restrictToRange(speed, minSpeed, maxSpeed);

            if (turnDegrees < 0)
                steer(-speed, speed);
            else
                steer(speed, -speed);

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
            // Reciprocal of maxRawPower so that when multiplied by ratio, it will equal 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }
        
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



    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget;
        double accuracyRange = 3;
        
        x = inchesToTicks(x);
        y = inchesToTicks(y);
        

        do {

            distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + Math.toRadians(preferredAngle);
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
            
            moveXYTurn(movement_x, movement_y, movement_turn);
            
        } while (distanceToTarget > -accuracyRange && distanceToTarget < accuracyRange);

    }
    
        
    
    public void myGoToPositionDeprecated(double xInches, double yInches, double maxSpeed) {
        double desiredX = inchesToTicks(xInches);
        double desiredY = inchesToTicks(yInches);
        double relativeXToTarget;
        double relativeYToTarget;
        double distanceToTarget;
        
        double absoluteAngleToTarget;
        
        double targetRange = 5;  // Distance in encoder ticks the robot must be from intended position to stop.
        
        
       do {
           relativeXToTarget = desiredX - worldXPosition;
           relativeYToTarget = desiredY - worldYPosition;
           distanceToTarget= Math.hypot(desiredX-worldXPosition, desiredY-worldYPosition);
           
           double leftSpeed = 0;
           double rightSpeed = 0;
           
           leftFront.setPower(leftSpeed);
           rightFront.setPower(rightSpeed);
           leftRear.setPower(leftSpeed);
           rightRear.setPower(rightSpeed);
           
        } while ((distanceToTarget > -targetRange && distanceToTarget < targetRange));
        
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