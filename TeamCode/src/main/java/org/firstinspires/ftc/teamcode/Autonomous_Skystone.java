package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.MalFunctionBot.DriveDirection;
import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG;
import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG_LEVELS;

@Autonomous(name="Autonomous!!!", group="Interative Opmode")
public class Autonomous_Skystone extends OpMode implements MalBase {

    // get robot and set up variables
    MalFunctionBot robot    = new MalFunctionBot();
    DEBUG_LEVELS INFO = DEBUG_LEVELS.INFO;
    DEBUG_LEVELS VERBOSE = DEBUG_LEVELS.VERBOSE;

    private ElapsedTime runtime = new ElapsedTime();

    HashMap<String, Telemetry.Item> telemetryItems = new HashMap<String, Telemetry.Item>();

    // auto run height variables
    boolean dpadReady = true;

    int scissorHeight = 0;

    int rot = 0;

    ConceptVuforiaSkyStoneNavigationNew detection = new ConceptVuforiaSkyStoneNavigationNew();

    int cam;

    @Override
    public void init() {
        // change the loop stuck detecters to be longer to prevent loop stuck crashes
        super.msStuckDetectLoop = 10000;
        super.msStuckDetectInit = 10000;

        initTelemetry();
        robot.init(hardwareMap, telemetryItems.get("status"), telemetry, this);

        cam = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        detection.startSensing(cam);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        runtime.reset();
        while (runtime.seconds() < 30) {
            // start moving

            // look for first skystone

            // keeps throwing error for getting stuck in this loop and also doesnt appear to be able to sense the blocks???

            while (!detection.skystoneVisible) {
                detection.senseTargets();
                robot.drive(DriveDirection.REVERSE, 0.5);
            }

            // look at skystone

            telemetry.addData("wow it worked", "");
            telemetry.update();


            if (detection.skystoneVisible) {
                detection.senseTargets();
                positionAtTarget();
            }

            // grab skystone

            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);

            robot.drive(MalFunctionBot.DriveDirection.REVERSE, 1);

            // strafe for alot

            robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 1);

            // servo up

            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_UP);

            // strafe back alot

            while (!detection.skystoneVisible) {
                detection.startSensing(cam);
                robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 0.5);
            }

            // look at skystone

            if (detection.skystoneVisible) {
                detection.startSensing(cam);
                positionAtTarget();
            }

            // grab skystone

            robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);

            robot.drive(MalFunctionBot.DriveDirection.REVERSE, 1);

            // strafe back alot

            robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 1);
        }
        robot.drive(DriveDirection.FORWARD, 0);
    }



    @Override
    // stop the opmode
    public void stop() {
        super.stop();
    }

    // push values to the telemetry
    public void log(Object value, String itemName, DEBUG_LEVELS level) {
        if (level.ordinal() >= DEBUG.ordinal()) {
            Telemetry.Item ti = this.telemetryItems.get(itemName);
            if(ti != null) {
                ti.setValue(value);
                telemetry.update();
            }
        }
    }

    // initialise the telemetry
    private void initTelemetry() {
        // checks which level of telemetry is called and pushes relevant data to the telemetry
        if(DEBUG_LEVELS.INFO.ordinal() >= DEBUG.ordinal()) {    // setup info level debug telemetry
            this.telemetryItems.put("status", telemetry.addData("status", ""));
        }
        if(DEBUG_LEVELS.VERBOSE.ordinal() >= DEBUG.ordinal()) {    // setup verbose level debug telemetry
            this.telemetryItems.put("ml_upperlimit", telemetry.addData("ml_upperlimit", ""));
            this.telemetryItems.put("ml_lowerlimit", telemetry.addData("ml_lowerlimit", ""));
            this.telemetryItems.put("dr_stick_ly", telemetry.addData("dr_stick_ly", ""));
            this.telemetryItems.put("dr_stick_lx", telemetry.addData("dr_stick_lx", ""));
        }
    }

    // function that uses the position and rotation values from Vuforia and the movement functions of the robot to position itself at the required locations on the field
    public void positionAtTarget(){
        while (detection.zRotation != 90 && detection.xPosition != 10 && detection.yPosition != 10) {
                robot.drive(MalFunctionBot.DriveDirection.TANK_ANTICLOCKWISE, 0.5);
            }
            while (detection.zRotation > 90) {
                robot.drive(MalFunctionBot.DriveDirection.TANK_CLOCKWISE, 0.5);
            }
            while (detection.xPosition < 10) {
                robot.drive(MalFunctionBot.DriveDirection.STRAFE_LEFT, 0.5);
            }
            while (detection.xPosition > 10) {
                robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 0.5);
            }
            while (detection.yPosition < 10) {
                robot.drive(MalFunctionBot.DriveDirection.REVERSE, 0.5);
            }
            while (detection.yPosition > 10) {
                robot.drive(MalFunctionBot.DriveDirection.FORWARD, 0.5);
            }
        }
    }
