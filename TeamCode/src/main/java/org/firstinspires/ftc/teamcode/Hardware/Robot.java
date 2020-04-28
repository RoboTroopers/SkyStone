package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Util.CustomTelemetry;
import org.firstinspires.ftc.teamcode.Util.Untested;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Util.MiscUtil.pause;
import static org.firstinspires.ftc.teamcode.Util.MyMath.angleWrapDeg;
import static org.firstinspires.ftc.teamcode.Util.MyMath.clampSigned;
import static org.firstinspires.ftc.teamcode.Util.MyMath.map;

@Untested
public class Robot {

    // Motors and servos
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    public List<DcMotor> base = new ArrayList<>();

    // Sensors
    public BNO055IMU imu;



    public final OpMode opMode;
    public final Telemetry telemetry;
    public final CustomTelemetry ct;


    public Robot(OpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        ct = new CustomTelemetry(this, telemetry);
    }


    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");

        base.add(leftFront);
        base.add(leftRear);
        base.add(rightFront);
        base.add(rightRear);

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) pause(15);
    }


    protected boolean opModeStopRequested() {
        if (opMode instanceof LinearOpMode) {
            return ((LinearOpMode) opMode).isStopRequested();
        }
        else return false;
    }


    public void setBaseMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: base) {
            motor.setMode(runMode);
        }
    }


    public void brake() {
        for (DcMotor motor: base) {
            motor.setPower(0);
        }
    }

    public void resetBase() {
        setBaseMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setBaseMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Untested
    public int getAvgMotorPos() {
        int pos = 0;
        for (DcMotor motor: base) {
            pos += motor.getCurrentPosition();
        }
        return pos/base.size();
    }


    public void setBase(double lf, double lr, double rf, double rr) {
        // Change motor powers only if they have changed
        if (leftFront.getPower() != lf)
            leftFront.setPower(lf);
        if (leftRear.getPower() != lr)
            leftRear.setPower(lr);

        if (rightFront.getPower() != rf)
            rightFront.setPower(rf);
        if (rightRear.getPower() != rr)
            rightRear.setPower(rr);
    }



    public void applyMovement(double straight, double strafe, double turn) {
        setBaseMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Move robot on field forward and sideways, rotates by turn
        //movement_x multiplied by 1.5 because mecanum drive strafes sideways slower than forwards/backwards
        double lf = straight - turn - (strafe * 1.5);
        double lr = straight - turn + (strafe * 1.5);
        double rf = straight + turn + (strafe * 1.5);
        double rr = straight + turn - (strafe * 1.5);

        // Find greatest power
        double maxRawPower = Math.max(Math.max(abs(lf), abs(rf)), Math.max(abs(lr), abs(rr)));

        double scaleDownFactor = 1.0;
        if (maxRawPower > 1.0) {
            // Reciprocal of maxRawPower so that when multiplied by factor, maxPower == 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }

        // Scale powers down equally to preserve direction, since direction is based on relative powers.
        lf *= scaleDownFactor;
        rf *= scaleDownFactor;
        lr *= scaleDownFactor;
        rr *= scaleDownFactor;

        setBase(lf, lr, rf, rr);

        pause(5);
    }



    /** Same principle as applyMovement but with encoder powers rather than motor speeds. */
    @Untested
    public void applyEncoderMovement(double straight, double strafe, double turn, double power) {
        final double minSpeed = 0.08; // Minimum speed kept to prevent steady-state error;
        // Cross your fingers this works.
        double lf_desired = straight - turn - (strafe * 1.5);
        double lr_desired = straight - turn + (strafe * 1.5);
        double rf_desired = straight + turn + (strafe * 1.5);
        double rr_desired = straight + turn - (strafe * 1.5);

        while (!opModeStopRequested()) {
            // Calculate motor encoder errors based on desired - actual formula.
            double lf_error = lf_desired - leftFront.getCurrentPosition();
            double lr_error = lr_desired - leftRear.getCurrentPosition();
            double rf_error = rf_desired - rightFront.getCurrentPosition();
            double rr_error = rr_desired - rightRear.getCurrentPosition();

            // Map motor speeds from encoder range to speed range, decreasing as error approaches 0;
            double lf = map(lf_error, lf_desired, 0, power, minSpeed);
            double lr = map(lr_error, lr_desired, 0, power, minSpeed);
            double rf = map(rf_error, rf_desired, 0, power, minSpeed);
            double rr = map(rr_error, rr_desired, 0, power, minSpeed);

            setBase(lf, lr, rf, rr);
        }
    }



    public double getWorldAngleDeg() { return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle); }



    // Tried to simplify this but idk if this is gonna work.
    @Untested
    public void turnToDeg(double desiredDeg, double maxSpeed) {
        brake();
        //resetBase();

        maxSpeed = abs(maxSpeed);
        final double initialAngle = getWorldAngleDeg();
        final double initialError = angleWrapDeg(desiredDeg - initialAngle); // Error = desired - actual;
        double errorDeg = initialError;

        final double desiredRange = 0.65;
        final double minSpeed = 0.115;
        final double deaccelRate = 2.85;

        while (Math.abs(errorDeg) > desiredRange && !opModeStopRequested()) {
            errorDeg = desiredDeg - getWorldAngleDeg();

            //double errorSign = errorDeg / abs(errorDeg);
            double errorRatio = abs(errorDeg) / abs(initialError);

            double turnSpeed = errorRatio * maxSpeed * deaccelRate;
            turnSpeed = clampSigned(turnSpeed, minSpeed, maxSpeed);

            applyMovement(0, 0, turnSpeed);

            telemetry.addData("Angle", getWorldAngleDeg());
            telemetry.addData("Raw Error (Deg)", desiredDeg - getWorldAngleDeg());
            telemetry.addData("Error (Deg)", errorDeg);
            telemetry.addData("InitialError", initialError);
            telemetry.addData("ErrorRatio", errorRatio);
            telemetry.addData("TurnSpeed", turnSpeed);
            telemetry.update();
        }

        brake();
    }


}
