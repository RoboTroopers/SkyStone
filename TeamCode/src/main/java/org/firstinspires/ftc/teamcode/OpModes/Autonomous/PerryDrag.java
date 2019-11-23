package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;





@Autonomous(name = "PerryDrag", group="Autonomous")
//@Disabled
public class PerryDrag extends LinearOpMode {
    
    public Robot perry = new Robot();


    @Override
    public void runOpMode() {
        
        perry.initHardware(hardwareMap);
        
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        
        waitForStart();
        
        
        if (opModeIsActive()) {

            perry.driveTrain.straightInches(-12, 1);
            perry.finger.down();
            perry.driveTrain.applyMovement(1, -0.3, -0.1);

            while (perry.driveTrain.getEncoderAvgInches() < inchesToTicks(TILE_LENGTH*2)) {
                telemetry.addData("Inches moved", perry.driveTrain.getEncoderAvgInches());
                pause(10);
            }
            
            perry.driveTrain.strafe(-1);
            while (perry.driveTrain.getEncoderAvgInches() < TILE_LENGTH*2) {}
            
            perry.driveTrain.brake();
            

        }
    }


}
