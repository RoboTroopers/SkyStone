package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {

    
    // Sensors
    public BNO055IMU imu;
    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    
    
    public void initHardware(HardwareMap aHwMap) {

        imu = aHwMap.get(BNO055IMU.class, "imu");
        horizontalEncoder = aHwMap.get(DcMotor.class, "horizontalEncoder");
        verticalEncoder = aHwMap.get(DcMotor.class, "verticalEncoder");
        resetEncoders();
        
    }
    

    public double getHorizontalEncoder() {
        return horizontalEncoder.getCurrentPosition();
    }

    public double getVerticalEncoder() {
        return verticalEncoder.getCurrentPosition();
        
    }
    
    
    public void resetEncoders() {
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
