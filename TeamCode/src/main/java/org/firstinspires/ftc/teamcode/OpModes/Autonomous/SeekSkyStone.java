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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Globals.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.FieldPosition;

import java.util.List;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;


/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Stone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */



@Autonomous(name = "Suck skystones", group="Autonomous")
//@Disabled
public class SeekSkyStone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    FieldPosition startPos = new FieldPosition(2, 0, DriveConstants.TRACK_WIDTH/2, WHEEL_BASE/2);
    public Robot robot = new Robot();
    //Thread odometryThread = new Thread(new OdometryThread(robot));

    private final double skystoneAngleOffset = 5; // How many degrees to add to skystone angle for robot to center on

    private int skystonesTransported = 0;

    private enum ProgramStates {

        SCANNING,
        APPROACHING,
        TRANSPORTING,
        PARKING

    }

    private ProgramStates currentState = ProgramStates.SCANNING;

    public double objectAngle;
    public double objectHeight;
    public double imageHeight;

    public double objectHeightRatio; // How much of the screen the skystone takes up vertically.



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

        robot.initHardware(hardwareMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Oof!", "This device is not compatible with TFOD");
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

            //Initialize everything the robot needs to start
            //robot.odometry.setPositionInches(startPos.fieldXInches, startPos.fieldYInches);
            //waitForStart();

            robot.driveTrain.straightInches(TILE_LENGTH-WHEEL_BASE, 0.7);

            //new Thread(odometryThread).start();

            while (opModeIsActive() && currentState != ProgramStates.PARKING) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.

                        Recognition nearestSkystone = getNearestStone(updatedRecognitions);

                        if (currentState == ProgramStates.SCANNING) {

                            if (nearestSkystone != null) {
                                // If skystone has been found
                                currentState = ProgramStates.APPROACHING;
                                robot.driveTrain.brake();
                            } else {
                                // If skystone has not been  found
                                telemetry.addData("Program State", "Scanning");

                                // Strafe left until Stone found within specific angle from center of camera
                                robot.driveTrain.strafe(-0.3);
                            }

                        } else if (currentState == ProgramStates.APPROACHING) {


                            if (nearestSkystone != null) {
                                objectAngle = nearestSkystone.estimateAngleToObject(AngleUnit.RADIANS);
                                objectHeight = nearestSkystone.getHeight();
                                imageHeight = nearestSkystone.getImageHeight();
                                objectHeightRatio = objectHeight / imageHeight; // Represents distance from skystone

                                telemetry.addData(String.format("  object height ratio"), "%.03f",
                                        objectHeightRatio);

                                telemetry.addData("Object height ratio", objectHeightRatio);
                                telemetry.addData("Object angle", objectAngle);


                                // The "1 - ([ratio])" is used to make robot slower when closer to skystone for precision.
                                double forwardSpeed = 0.3*(1 - (objectHeightRatio));
                                double strafeSpeed = (objectAngle+skystoneAngleOffset)*0.15;

                                telemetry.addData("Program State", "Approaching ");
                                // Move towards the skystone until it takes up enough of the screen, meaning it is close enough to pick up.

                                robot.driveTrain.applyMovement(strafeSpeed, forwardSpeed, 0);
                                robot.intake.suck();

                            } else {
                                //Robot cannot recognize skystone any longer when it is close because camera is looking over it.
                                robot.driveTrain.brake();
                                currentState = ProgramStates.TRANSPORTING;
                            }

                        } else if (currentState == ProgramStates.TRANSPORTING) {

                            /*while (!robot.sensors.possessingStone()) {
                                robot.driveTrain.straightInches(10, 0.5);
                            }*/

                            robot.driveTrain.straightInches(4, 0.6);

                            telemetry.addData("Ladies and gentlemen!", "We gottem.");

                            sleep(1500);
                            skystonesTransported += 1;
                            robot.intake.rest();
                            robot.driveTrain.brake();
                            robot.driveTrain.straightInches(-10, 0.7);

                            robot.driveTrain.turnToRad(toRadians(-90), 0.6, 6);
                            robot.driveTrain.straightInches(TILE_LENGTH*3, 5);

                            // Release skystone onto ground
                            robot.outtake.depositStone();

                            robot.driveTrain.straightInches(-TILE_LENGTH*3, 5);
                            robot.driveTrain.turnToRad(0, 0.6, 6);

                            if (skystonesTransported < 2) {
                                currentState = ProgramStates.SCANNING;

                            } else {
                                currentState = ProgramStates.PARKING;
                            }

                            break;

                        }
                    }
                }

                telemetry.update();
            }

            if (currentState == ProgramStates.PARKING) {

                telemetry.addData("Program State", "Parking");
                robot.driveTrain.strafe(0.5);

                //while (!robot.sensors.overLine()) {
                telemetry.addData("Parking...", true);
                //}

                robot.driveTrain.brake();
                //robot.advancedMovement.myGoToPosition(BRIDGE_X, TILE_LENGTH, 0.6, 0, 0.5);

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }



    /**
     * Initialize the Vuforia localization engine.
     */    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


/*
    @Deprecated
    private void initVuforiaWebcam() {

         // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }*/



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



    private Recognition getNearestStone(List <Recognition> updatedRecognitions) {

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

        return nearestSkystone;
    }


}
