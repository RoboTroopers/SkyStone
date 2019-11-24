package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;





@Autonomous(name = "PerryPark", group="Autonomous")
//@Disabled
public class PerryPark extends LinearOpMode {
    
    public Robot perry = new Robot();


    @Override
    public void runOpMode() {
        
        perry.initHardware(hardwareMap);
        
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        
        waitForStart();
        
        
        if (opModeIsActive()) {
            
            perry.driveTrain.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
            int distance = (int)Math.round(TILE_LENGTH);
            perry.driveTrain.setTargetPos(distance);
            
            while (perry.driveTrain.getEncoderAvgInches() < distance);
            perry.driveTrain.straight(0.5);
        }
        perry.driveTrain.brake();
    }


}
