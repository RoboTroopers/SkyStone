package org.firstinspires.ftc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Bot extends OpMode
{
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public BNO055IMU imu;

    public DcMotor xEncoder;
    public DcMotor yEncoder;

    private double worldXPos = 0;
    private double worldYPos = 0;

    private double xEncoderLast = 0;
    private double yEncoderLast = 0;


    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        xEncoder = hardwareMap.dcMotor.get("xEncoder");
        yEncoder = hardwareMap.dcMotor.get("yEncoder");

    }

    public double getWorldAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        // Since the imu is rotated on its x-axis 90 degrees, the x-axis is the one that points up.
        return angles.firstAngle;
    }

    public double getWorldXPos()
    {
        return worldXPos;
    }

    public double getWorldYPos()
    {
        return worldYPos;
    }

    public void updatePos()
    {
        double relativeXChange = xEncoder.getCurrentPosition() - xEncoderLast;
        double relativeYChange = yEncoder.getCurrentPosition() - yEncoderLast;

        double worldAngleRad = Math.toRadians(getWorldAngle());

        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = relativeYChange * Math.cos(worldAngleRad);
        double forwardShiftY = relativeYChange * Math.sin(worldAngleRad);

        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = relativeXChange * Math.cos(worldAngleRad);
        double strafeShiftY = relativeXChange * Math.sin(worldAngleRad);

        worldXPos += forwardShiftX + strafeShiftX;
        worldYPos += forwardShiftY + strafeShiftY;

        xEncoderLast = xEncoder.getCurrentPosition();
        yEncoderLast = xEncoder.getCurrentPosition();
    }


    public void applyMovement(double forward, double sideways, double turn)
    {
        //Moves robot on field forward and sideways, rotates by turn
        //movement_x multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double frontLeftRaw = forward + turn - (sideways*1.5);
        double frontRightRaw = forward - turn + (sideways*1.5);
        double rearLeftRaw = forward + turn + (sideways*1.5);
        double rearRightRaw = forward - turn - (sideways*1.5);

        // Find greatest power
        double maxRawPower = Math.max(Math.max(frontLeftRaw, frontRightRaw), Math.max(rearLeftRaw, rearRightRaw));

        double scaleDownFactor = 1.0;
        if (maxRawPower > 1.0)
        {
            // Reciprocal of maxRawPower so that when multiplied by factor, maxPower == 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }

        // All motor speeds scaled down (if maxRawPower > 1) but vector is preserved.
        frontLeftRaw *= scaleDownFactor;
        frontRightRaw *= scaleDownFactor;
        rearLeftRaw *= scaleDownFactor;
        rearRightRaw *= scaleDownFactor;

        // Changes motor powers only if they have changed
        if (frontLeft.getPower() != frontLeftRaw)
        {
            frontLeft.setPower(frontLeftRaw);
        }
        if (frontRight.getPower() != frontRightRaw)
        {
            frontRight.setPower(frontRightRaw);
        }
        if (rearLeft.getPower() != rearLeftRaw)
        {
            rearLeft.setPower(rearLeftRaw);
        }
        if (rearRight.getPower() != rearRightRaw)
        {
            rearRight.setPower(rearRightRaw);
        }
    }


    public void glideToPos(double xPos, double yPos, double xSpeed, double ySpeed, double preferredAngle, double turnSpeed)
    {
        double absoluteDist = Math.hypot(xPos - worldXPos, yPos - worldYPos);
        double absoluteAngle = Math.atan2(yPos - worldYPos, xPos - worldXPos);
        double relativeAngle = MathMethods.angleWrap(absoluteAngle - (getWorldAngle() - Math.toRadians(90)));

        double relativeX = Math.cos(relativeAngle) * absoluteDist;
        double relativeY = Math.sin(relativeAngle) * absoluteDist;

        double denominator =  Math.abs(relativeX) + Math.abs(relativeY);

        double xPower = relativeX / denominator;
        double yPower = relativeY / denominator;


        double turnPower = relativeAngle - Math.toRadians(180) + preferredAngle;
        turnPower = MathMethods.clamp(turnPower/Math.toRadians(30), -1, 1);

        final double turnSlowThreshold = 10;
        if (absoluteDist < turnSlowThreshold)
        {
            turnPower *= absoluteDist / turnSlowThreshold;
        }

        applyMovement(yPower * ySpeed, xPower * xSpeed, turnPower * turnSpeed);
    }


    @Override
    public void loop()
    {
        updatePos();
        UtilMethods.delay(15);
    }
}
