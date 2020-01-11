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
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeTypes;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;

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



@Autonomous(name = "Seek Skystones NEW", group="Autonomous")
//@Disabled
public class SeekSkyStone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public Robot robot = new Robot(this, OpModeTypes.AUTO);
    //Thread odometryThread = new Thread(new OdometryThread(robot));

    private int skystonesDelivered = 0;

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
        initVuforiaWebcam();

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
            // robot.odometry.setPositionInches(startPos.fieldXInches, startPos.fieldYInches);
            //waitForStart();

            //robot.driveTrain.strafe(0.3);
            robot.driveTrain.straightInches(TILE_LENGTH*1, 0.1);

            //new Thread(odometryThread).start();

            while (skystonesDelivered < 2 && opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    Recognition nearestSkystone = getNearestSkystone();

                    currentState = ProgramStates.SCANNING;

                    while (nearestSkystone == null) {

                        nearestSkystone = getNearestSkystone();

                        telemetry.addData("Program State", "Scanning");
                        // Strafe left until Stone found within specific angle from center of camera
                        //robot.driveTrain.strafeInches(STONE_WIDTH, -0.2);
                        //robot.driveTrain.straightInches(STONE_WIDTH, 0.05);

                        robot.driveTrain.strafe(-0.25);
                    }

                    // If skystone has been found
                    currentState = ProgramStates.APPROACHING;
                    robot.driveTrain.brake();

                    objectAngle = nearestSkystone.estimateAngleToObject(AngleUnit.DEGREES);
                    objectHeight = nearestSkystone.getHeight();
                    imageHeight = nearestSkystone.getImageHeight();
                    objectHeightRatio = objectHeight / imageHeight; // Represents distance from skystone

                    telemetry.addData("Program State", "Approaching ");
                    telemetry.addData("Object height ratio", objectHeightRatio);
                    telemetry.addData("Object angle", objectAngle);
                    telemetry.update();

                    // The "1 - ([ratio])" is used to make robot slower when closer to skystone for precision.
                    //double forwardSpeed = 0.3*(1 - (objectHeightRatio));
                    //double strafeSpeed = (objectAngle+skystoneAngleOffset)*0.15;

                    // Go sideways (since robot is sideways, forward) proportional to angle of stone in camera.
                    //double forwardSpeed = ((objectAngle+skystoneAngleOffset)/45)*0.5;
                    // Set vertical speed proportional to distance from skystone.
                    //double strafeSpeed = 0.5*(1 - (objectHeightRatio))+0.12;

                    robot.driveTrain.applyMovement(0.18,-0.15, -0.11);
                    sleep(1000);
                    robot.driveTrain.brake();

                    robot.intake.setSpeed(0.45);
                    //robot.driveTrain.straightInches(TILE_LENGTH*0.3, 0.05);
                    robot.driveTrain.straight(0.20);
                    sleep(1500);
                    robot.driveTrain.brake();

                    sleep(500);
                    currentState = ProgramStates.TRANSPORTING;

                    robot.intake.rest();
                    telemetry.addData("Ladies and gentlemen!", "We gottem.");
                    skystonesDelivered += 1;

                    robot.driveTrain.turn(-0.155);
                    sleep(1000);

                    robot.intake.rest();


                    robot.driveTrain.straightInches(20, 0.1);
                    sleep(500);

                    robot.driveTrain.strafe(0.3);
                    if(skystonesDelivered == 1)
                        sleep(8000);
                    else
                        sleep(16000);

                    robot.driveTrain.brake();
                    robot.intake.blow();

                    sleep(2000);
                    robot.intake.rest();

                    robot.driveTrain.applyMovement(0, -0.4, -0.01);
                    sleep(3000);
                    robot.driveTrain.brake();

                }
            }

            currentState = ProgramStates.PARKING;

            robot.sensors.lineSensor.enableLed(true);

            telemetry.addData("Program State", "Parking");
            robot.driveTrain.brake();

            while (!robot.sensors.overLine()) {
                telemetry.addData("LightSensor", robot.sensors.getColorSensorHSV(robot.sensors.lineSensor));
                telemetry.update();
            }

            robot.driveTrain.brake();
            //robot.advancedMovement.myGoToPosition(BRIDGE_X, TILE_LENGTH, 0.6, 0, 0.5);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }



    private void initVuforiaWebcam() {

         // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);// idk but this might fix the problem of the robot going for normal stones
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SECOND_ELEMENT, LABEL_SECOND_ELEMENT);

    }



    private Recognition getNearestSkystone() {

        Recognition nearestSkystone = null;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.

            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());

                telemetry.addData(String.format("  estimated angle (%d)", i), "%.03f",
                        recognition.estimateAngleToObject(AngleUnit.DEGREES));

                // Gets the nearest skystone (largest height on the screen) to the robot.
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    if (nearestSkystone != null) {
                        // If the previous nearest skystone has been declared and is farther than current recognition, set nearest skystone to current recognition.
                        if (recognition.getHeight() > nearestSkystone.getHeight()) {
                            nearestSkystone = recognition;
                        }
                        //If current recognition is the first skystone recognized, then use it as the nearest skystone.
                    } else nearestSkystone = recognition;
                }

            }
        }
        return nearestSkystone;
    }

}


