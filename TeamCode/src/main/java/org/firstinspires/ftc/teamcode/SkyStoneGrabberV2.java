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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="SKYSTONE GRABBER v2!!!!!", group ="Concept")
public class SkyStoneGrabberV2 extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

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
            "AYkacKP/////AAABmQQpHOjXFUmUrnC2GiZindRweLeZxahXKOvfR8FRBguC5RmdDL76ZJ2czVqjXEDl1DzxVO2/7pKVhYZEDOAFBqKkYTGpWc73dm/HBuFATzQiADhxyL25qqAT1FAmrzGg/+2RspZxHin1jnPp6v70BMxpCfhjf+Tv157CB6raEc15lKQyMZjvrONlylhFUiSZ0Vc5zHTsWfDPD87vrtdVJ4DEtIBRHCtG9FIk1pHSsx0TFF97Avd5ujqj/O40OJN8mcrvmYowtSJPPaIcYogddHyk2BFRF5XahIThhhhhJ/VoTa3gDEsRrBLxdm2RMWRAQUCSjPguRNNSu5Cc4T2RsXVMAUEKNCIgLRG/vHD3UFPN";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    // target visibility variables
    public boolean skystoneVisible = false;
    public boolean backTargetVisible = false;
    public boolean sideTargetVisible = false;

    // relative position variables
    public double xPosition = 0;
    public double yPosition = 0;
    public double zPosition = 0;
    public double zRotation = 0;

    MalFunctionBot robot    = new MalFunctionBot();
    MalGlobals.DEBUG_LEVELS INFO = MalGlobals.DEBUG_LEVELS.INFO;
    MalGlobals.DEBUG_LEVELS VERBOSE = MalGlobals.DEBUG_LEVELS.VERBOSE;

    private ElapsedTime runtime = new ElapsedTime();

    HashMap<String, Telemetry.Item> telemetryItems = new HashMap<String, Telemetry.Item>();

    // auto run height variables
    boolean dpadReady = true;

    boolean keepLooking = true;

    int scissorHeight = 0;

    int rot = 0;

    public enum WhatNow {
        LOOK,
        REVERSELOOK,
        GRAB,
        DRIVETOSIDE,
        POSITION
    }

    int[] stoneDistance;

    int actualStoneDistance;

    WhatNow next = WhatNow.LOOK;

    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        // change the loop stuck detecters to be longer to prevent loop stuck crashes
        super.msStuckDetectLoop = 10000;
        super.msStuckDetectInit = 10000;

        initTelemetry();
        robot.init(hardwareMap, telemetryItems.get("status"), telemetry, this);

        runtime.reset();

        targetsSkyStone.activate();
        while (!isStopRequested()) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    // telemetry.addData("Visible Target", trackable.getName());

                    // set specific target visible variables
                    if (trackable.getName().equals("Stone Target")) {
                        // .addLine("Stone target is visible");
                        skystoneVisible = true;
                    } else {
                        skystoneVisible = false;
                    }

                    if (trackable.getName().equals("Rear Perimeter 1")) {
                        // telemetry.addLine("Back target is visible");
                        backTargetVisible = true;
                    } else {
                        backTargetVisible = false;
                    }

                    if (trackable.getName().equals("Red Perimeter 1")) {
                        // telemetry.addLine("Side target is visible");
                        sideTargetVisible = true;
                    } else {
                        sideTargetVisible = false;
                    }

                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            String positionSkystone = "";
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                // telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                xPosition = translation.get(0);
                yPosition = translation.get(1);
                zPosition = translation.get(2);

//                if (xPosition < -10){
//                    positionSkystone = "left";
//                } else {
//                    positionSkystone = "center";
//                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                // telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                zRotation = rotation.thirdAngle;
            } else {
//                positionSkystone = "right";
                // telemetry.addData("Visible Target", "none");
            }

            // don't do this here, it needs to be in the init method
            // runtime.reset();

            telemetry.addData("Mal Is Doing: ", next);
            telemetry.addData("xPos: ", xPosition);
            telemetry.addData("yPos: ", yPosition);
            telemetry.addData("zPos: ", zPosition);
            telemetry.addData("zRot: ", zRotation);
            telemetry.update();

            switch (next) {
                case LOOK:
                    look(MalFunctionBot.DriveDirection.REVERSE);
                    break;
                case REVERSELOOK:
                    look(MalFunctionBot.DriveDirection.FORWARD);
                    break;
                case GRAB:
                    grab();
                    break;
                case DRIVETOSIDE:
                    break;
                case POSITION:
                    position();
                    break;
                default:
                    break;
            }

            // ********************
            // ** it's already looping so you don't need the while loops.
            // ********************


//        while (runtime.seconds() < 30) {
//            // start moving
//
//            // look for first skystone
//
//            // keeps throwing error for getting stuck in this loop and also doesnt appear to be able to sense the blocks???
//
//            while (!detection.skystoneVisible) {
//                detection.senseTargets();
//                robot.drive(DriveDirection.REVERSE, 0.5);
//            }
//
//            // look at skystone
//
//            telemetry.addData("wow it worked", "");
//            telemetry.update();
//
//
//            if (detection.skystoneVisible) {
//                detection.senseTargets();
//                positionAtTarget();
//            }
//
//            // grab skystone
//
//            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);
//
//            robot.drive(MalFunctionBot.DriveDirection.REVERSE, 1);
//
//            // strafe for alot
//
//            robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 1);
//
//            // servo up
//
//            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_UP);
//
//            // strafe back alot
//
//            while (!detection.skystoneVisible) {
//                detection.startSensing(cam);
//                robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 0.5);
//            }
//
//            // look at skystone
//
//            if (detection.skystoneVisible) {
//                detection.startSensing(cam);
//                positionAtTarget();
//            }
//
//            // grab skystone
//
//            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);
//
//            robot.drive(MalFunctionBot.DriveDirection.REVERSE, 1);
//
//            // strafe back alot
//
//            robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 1);
//        }
            // robot.drive(DriveDirection.FORWARD, 0);
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    // initialise the telemetry
    private void initTelemetry() {
        // checks which level of telemetry is called and pushes relevant data to the telemetry
        if(MalGlobals.DEBUG_LEVELS.INFO.ordinal() >= DEBUG.ordinal()) {    // setup info level debug telemetry
            this.telemetryItems.put("status", telemetry.addData("status", ""));
        }
        if(MalGlobals.DEBUG_LEVELS.VERBOSE.ordinal() >= DEBUG.ordinal()) {    // setup verbose level debug telemetry
            this.telemetryItems.put("ml_upperlimit", telemetry.addData("ml_upperlimit", ""));
            this.telemetryItems.put("ml_lowerlimit", telemetry.addData("ml_lowerlimit", ""));
            this.telemetryItems.put("dr_stick_ly", telemetry.addData("dr_stick_ly", ""));
            this.telemetryItems.put("dr_stick_lx", telemetry.addData("dr_stick_lx", ""));
        }
    }

    private void look(MalFunctionBot.DriveDirection dir) {
        if(skystoneVisible) {
            // do visible stuff like drive to it
            telemetry.addData("wow it worked", "");
            telemetry.update();
            next = WhatNow.POSITION;
        } else {
            // can't see it so do something
            robot.drive(dir, 0.2);
        }
    }

    private void position() {
        if (runtime.seconds() % 2 == 0)
        while (runtime.seconds() - (int)runtime.seconds() < 0.2) {
            robot.drive(MalFunctionBot.DriveDirection.STOP, 0);
        }
        if (xPosition < -400) {
            robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 0.8);
        } else if (xPosition > -460) {
            robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 0.8);
        } else if (yPosition < -220) {
            robot.drive(MalFunctionBot.DriveDirection.FORWARD, 0.8);
        } else if (yPosition > -150) {
            robot.drive(MalFunctionBot.DriveDirection.REVERSE, 0.8);
        } else if (zRotation < -6) {
            robot.drive(MalFunctionBot.DriveDirection.TANK_ANTICLOCKWISE, 0.5);
        } else if (zRotation > 6) {
            robot.drive(MalFunctionBot.DriveDirection.TANK_CLOCKWISE, 0.5);
        } else {
            robot.drive(MalFunctionBot.DriveDirection.STOP, 0);
            next = WhatNow.GRAB;
        }
    }

    private void grab() {
        robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);
        robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 1); // until hits wall
        // robot.drive(MalFunctionBot.DriveDirection.FORWARD, 1); // until in opposite side
    }
}
