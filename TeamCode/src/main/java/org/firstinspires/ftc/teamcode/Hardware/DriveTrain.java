package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.MiscUtil;

public class DriveTrain {
    
    
    // Motors and servos
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    public enum MoveStates {

        STATIONARY,
        STRAIGHT,
        STEERING,
        PIVOTING,
        STRAFING,
        APPLYING_MOVEMENT
        
    }
    
    
    MoveStates moveState = MoveStates.STATIONARY;
    
    
    public void initHardware(HardwareMap aHwMap) {
        
        leftFront = aHwMap.get(DcMotor.class, "leftFront");
        rightFront = aHwMap.get(DcMotor.class, "rightFront");
        leftRear = aHwMap.get(DcMotor.class, "leftRear");
        rightRear = aHwMap.get(DcMotor.class, "rightRear");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
    }
    
    
    // Stations the robot in current position
    public void brake() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        
        moveState = MoveStates.STATIONARY;
        
    }
    
    
    // Moves all motors at same power
    public void straight(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        
        moveState = MoveStates.STRAIGHT;
        
    }
    
    
    // Moves the left and right side motors separate speeds
    public void steer(double leftSpeed, double rightSpeed) {
        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(rightSpeed);
        
        if (leftSpeed == rightSpeed) {
            moveState = MoveStates.STRAIGHT;
        
        } else if (leftSpeed == -rightSpeed) {
            moveState = MoveStates.PIVOTING;
        
        } else {
            moveState = MoveStates.STEERING;
        }
        
    }
    
    
    // Moves the robot sideways without turning
    public void strafe(double speed){
        // Positive speed strafes right, negative speed strafes left.
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(-speed);
        
        moveState = MoveStates.STRAFING;
        
    }
    
    
    public void applyMovement(double horizontal, double vertical, double turn) {
        
        //Moves robot on field horizontally and vertically, rotates by turn
        MiscUtil.pause(15);
        
        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double lf_power_raw = vertical + turn - (horizontal*1.5);
        double lr_power_raw = vertical + turn + (horizontal*1.5);
        double rf_power_raw = vertical - turn + (horizontal*1.5);
        double rr_power_raw = vertical - turn - (horizontal*1.5);


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

        moveState = MoveStates.APPLYING_MOVEMENT;
        
    }
    
    
}
