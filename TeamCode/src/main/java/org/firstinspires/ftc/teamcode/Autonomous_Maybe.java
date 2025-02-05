package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.MalFunctionBot.ArmDirection;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.ArmPosition;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.DriveDirection;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.FORWARD;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.GripperDirection;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.LiftDirection;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.ServoDirection;
import static org.firstinspires.ftc.teamcode.MalFunctionBot.SpinnerRotationDirection;
import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG;
import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG_LEVELS;

@Autonomous(name="Autonomous!", group="Interative Opmode")
public class Autonomous_Maybe extends OpMode implements MalBase {

    // get robot and set up variables
    MalFunctionBot robot    = new MalFunctionBot();
    DEBUG_LEVELS INFO = DEBUG_LEVELS.INFO;
    DEBUG_LEVELS VERBOSE = DEBUG_LEVELS.VERBOSE;

    private ElapsedTime runtime = new ElapsedTime();

    HashMap<String, Telemetry.Item> telemetryItems = new HashMap<String, Telemetry.Item>();

    // auto run height variables
    boolean dpadReady = true;

    int scissorHeight = 0;

    @Override
    public void init() {
        // change the loop stuck detecters to be longer to prevent loop stuck crashes
        super.msStuckDetectLoop = 10000;
        super.msStuckDetectInit = 10000;

        initTelemetry();
        robot.init(hardwareMap, telemetryItems.get("status"), telemetry, this);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.drive(DriveDirection.FORWARD, 0.5);
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
