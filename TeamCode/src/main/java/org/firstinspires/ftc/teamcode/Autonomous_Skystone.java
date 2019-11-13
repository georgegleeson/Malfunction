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

    TargetDetection detection = new TargetDetection();

    @Override
    public void init() {
        // change the loop stuck detecters to be longer to prevent loop stuck crashes
        super.msStuckDetectLoop = 10000;
        super.msStuckDetectInit = 10000;

        initTelemetry();
        robot.init(hardwareMap, telemetryItems.get("status"), telemetry, this);

        detection.startSensing();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        while (runtime.seconds() < 30) {
            // start moving

            // look for first skystone

            while (!detection.skystoneVisible) {
                robot.drive(DriveDirection.REVERSE, 0.5);
            }

            // look at skystone

            if (detection.skystoneVisible) {
                detection.positionAtTarget();
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
                robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 0.5);
            }

            // look at skystone

            if (detection.skystoneVisible) {
                detection.positionAtTarget();
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
    public void loop() {

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
}
