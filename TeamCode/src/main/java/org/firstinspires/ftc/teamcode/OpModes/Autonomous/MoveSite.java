package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.Utilities.MiscUtil.pause;





@Autonomous(name = "MoveSite", group="Autonomous")
//@Disabled
public class MoveSite extends LinearOpMode {
    
    public Robot robot = new Robot();


    @Override
    public void runOpMode() {
        
        robot.initHardware(hardwareMap);
        
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        
        waitForStart();
        
        
        if (opModeIsActive()) {

            robot.driveTrain.straightInches(-12, 1);
            robot.finger.down();
            robot.driveTrain.applyMovement(1, -0.3, -0.1);

            while (robot.driveTrain.getEncoderAvgInches() < inchesToTicks(TILE_LENGTH*2)) {
                telemetry.addData("Inches moved", robot.driveTrain.getEncoderAvgInches());
                pause(10);
            }
            
            robot.driveTrain.strafe(-1);
            while (robot.driveTrain.getEncoderAvgInches() < TILE_LENGTH*2) {}
            
            robot.driveTrain.brake();
            

        }
    }


}
