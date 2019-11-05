/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Odometry.OdometryThread;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.DriveConstants;
import org.firstinspires.ftc.teamcode.Utilities.FieldPosition;

import java.util.List;


/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */



@Autonomous(name = "Tensorflow Skystone Detection Autonomous", group="Autonomous")
//@Disabled
public class SeekSkyStone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    
    FieldPosition position = new FieldPosition(2, 0, DriveConstants.TRACK_WIDTH/2, DriveConstants.WHEEL_BASE/2);
    public Robot robot = new Robot(0,0,0);
    OdometryThread odometryThread = new OdometryThread(robot);
    
    // Distance from the center of the screen that the skystone can be to pick it up (degrees)
    private final double skystoneAngleTolerance = 15;
    
    // How much of the screen the skystone needs to take up for the robot to deploy the pinger
    private final double desiredHeightRatio = 0.8;
    
    //How far to the side the skystone should be (degrees)
    public final double desiredHorizontalAngle = 0;
    
    
    public int skystonesTransported = 0;
    public boolean lockedOn = false;
    public boolean transporting = false;

    
    public enum ProgramState {
        SEARCHING,
        LOCKED_ON,
        TRANSPORTING,
        FINALIZING
        
    }


    public double objectAngle;
    public double objectHeight;
    public double imageHeight;

    // How much of the screen the skystone takes up vertically.
    public double objectHeightRatio;

    public double speedMultiplier;
    public double forwardSpeed;
    public double turnSpeed;
    
    


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQjY1NP/////AAABmUvaVtQ0nUQ9tejvctez83szc6mfruVEZTBCKtHg2fP0Mj/JZi9/l7fdKbXD9311fPDo7mIzkBaV6RcWT5LY5ksEfoUJXc/ewDYGpkB08zWSHn0C6cP8A2Dxak5l+WsHht7b12+aitu5fDbmIZ8zwtwJ6Lxu3OynVEt95+MfVjfQF2qpSfS0FtgBQMkkBBlTxZPaCkX1/4HJqcZokwgrUZMH5UBvNtSxveBKyHEMznVJiHg3gw6drdIOgfw/+mgdS3Il7MXwMHd13Fm7Un7wyrfcMxOXSqfOOaAymMOCLRQNDUUBJFZF2/QPWZnHHZzEE/nZo7uARlDXDM8aL+JB+chJa9ipx5hhBrvBg7z839Wz";
    
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    
    
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        
        
        if (opModeIsActive()) {
            
            robot.initHardware(hardwareMap);
            waitForStart();
            
            new Thread(odometryThread).start();
            
            while (opModeIsActive()) {
                
                // Strafe right until Skystone found within threshold
                if (skystonesTransported < 2)
                    robot.driveTrain.strafe(-0.5);
                else {
                    robot.driveTrain.brake();
                }
                
                while (skystonesTransported < 2) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            
                            int i = 0;
                            Recognition nearestSkystone = null;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                
                                telemetry.addData(String.format("  estimated angle (%d)", i), "%.03f",
                                        recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                
                                // Gets the nearest skystone (largest height on the screen) to the robot.
                                if (nearestSkystone != null) {
                                    // If the previous nearest skystone has been declared and is farther than current recognition, set nearest skystone to current recognition.
                                    if (recognition.getHeight() > nearestSkystone.getHeight())
                                        nearestSkystone = recognition;
                                    //If current recognition is the first skystone recognized, then use it as the nearest skystone.
                                } else nearestSkystone = recognition;
                            }
                            
                            // Set angle based on how far from the center and which side of the camera the stone is.
                            objectAngle = nearestSkystone.estimateAngleToObject(AngleUnit.DEGREES);
                            objectHeight = nearestSkystone.getHeight();
                            imageHeight = nearestSkystone.getImageHeight();
                            objectHeightRatio = objectHeight/imageHeight;
                            
                            telemetry.addData(String.format("  object height ratio (%d)", i), "%.03f",
                                    objectHeightRatio);
                            
                            
                            // If skystone angle is within threshold, brake and prepare to make fine adjustments
                            if (Math.abs(objectAngle) < skystoneAngleTolerance && !lockedOn && !transporting) {
                                robot.driveTrain.brake();
                                lockedOn = true;
                            }
                            
                            if (lockedOn) {
                                
                                forwardSpeed = 0.5;
                                turnSpeed = (objectAngle/45)*0.5;
                                
                                // The "1 - ([ratio])" is used to make robot slower when closer to skystone for precision.
                                speedMultiplier = (0.25*(1-(objectHeightRatio/desiredHeightRatio)))+0.25;
                                
                                // Move towards the skystone until the object takes up enough of the screen.
                                // This is when the robot is at the optimal distance to use the pinger.
                                // Also turn simulateously towards skystone while moving forward.
                                robot.driveTrain.steer((forwardSpeed+turnSpeed)*speedMultiplier, (forwardSpeed-turnSpeed)*speedMultiplier); 
                                
                                if (objectHeightRatio > desiredHeightRatio) {
                                    transporting = true;
                                    lockedOn = false;
                                }
                            }
                            
                            if (transporting) {
                                
                                // Extend pinger outward to turn the skystone 90 degrees to prepare the skystone for The Succ.
                                robot.intake.pingerOut();
                                sleep(1000);
                                robot.intake.pingerIn();
                                robot.driveTrain.steer(0.5, 0.5);
                                sleep(2000);
                                robot.driveTrain.brake();
                                telemetry.addData("Ladies and gentlemen!", "We gottem.");
                                //robot.goToPosition(30, 20, 0.2, 0, 0.4);
                                //robot.goToPosition(0, 0, 0.3, 0, 0.4);
                                
                                skystonesTransported += 1;
                                transporting = false;
                                continue;
                                
                            }
                            
                            telemetry.update();
                            
                        }
                    }
                }
            }
        }
        
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    
    
}
