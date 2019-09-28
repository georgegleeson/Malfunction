package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.MalGlobals.*;

public class MalFunctionBot {
    // Motors
    private DcMotor rfDrive  = null;     // Front Right
    private DcMotor rrDrive  = null;     // Rear Right
    private DcMotor lfDrive  = null;     // Front Left
    private DcMotor lrDrive  = null;     // Rear Left
    private DcMotor mainLift = null;     // scissor lift

    // Servos
    private Servo serv = null;

    // Switches
    private DigitalChannel mainLiftUpperLimit;
    private DigitalChannel mainLiftLowerLimit;

    // REV IMU
    private BNO055IMU imu   = null;

    // Telemetry object
    private Telemetry telemetry         = null;
    private Telemetry.Item teleStatus   = null;

    // Telementry message
    public String message = "";

    // Robot init completed
    public boolean initComplete = false;

    // Robot components
    public HashMap<String, Boolean> robotConfig = new HashMap<String, Boolean>();

    // Robot hardware map
    HardwareMap hwMap       = null;

    static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;

    public enum DriveDirection {
        FORWARD,
        REVERSE,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TANK_CLOCKWISE,
        TANK_ANTICLOCKWISE,
        STOP
    }

    public enum LiftDirection {
        UP,
        DOWN,
        STOP
    }

    public MalFunctionBot() {
        robotConfig.put("IMU", false);
        robotConfig.put("drive", true);
        robotConfig.put("scissorLift", true);
    }

    public void init(HardwareMap ahwMap, Telemetry.Item ti, Telemetry telemetry) {
        this.teleStatus = ti;
        this.telemetry = telemetry;
        setStatus("Mal is waking up");

        hwMap = ahwMap;

        try {
            // ensure the order is set
            if (robotConfig.get("drive")) initDrive();
            if (robotConfig.get("scissorLift")) initMainLift();
            if (robotConfig.get("IMU")) initIMU();

            setStatus("Mal is ready to rock");
            this.initComplete = true;
        } catch (Exception e) {
            this.message = e.toString();
        }
    }

    public boolean mainLiftUpperLimitState() { return mainLiftUpperLimit.getState(); }
    public boolean mainLiftLowerLimitState() { return mainLiftLowerLimit.getState(); }

    public void moveMainLift(LiftDirection direction) {
        switch (direction){
            case UP:
                if(!mainLiftUpperLimitState()) {
                    mainLift.setDirection(FORWARD);
                    mainLift.setPower(1);
                } else mainLift.setPower(0);
                break;
            case DOWN:
                if(!mainLiftLowerLimitState()) {
                    mainLift.setDirection(REVERSE);
                    mainLift.setPower(1);
                } else mainLift.setPower(0);
                break;
            default:
                mainLift.setPower(0);
        }
    }

    public void drive(double leftStickX, double leftStickY, double rightStickX) {
      double r = Math.hypot(leftStickX, leftStickY);
      double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
      final double p1 = r * Math.cos(robotAngle) + rightStickX;
      final double p2 = r * Math.sin(robotAngle) - rightStickX;
      final double p3 = r * Math.sin(robotAngle) + rightStickX;
      final double p4 = r * Math.cos(robotAngle) - rightStickX;

      lfDrive.setPower(p1);
      rfDrive.setPower(p2);
      lrDrive.setPower(p3);
      rrDrive.setPower(p4);
    }

    private void initIMU() {
        setStatus("Initialising IMU");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        imu = hwMap.get(BNO055IMU.class, "imu");

        if (imu != null) {
            imu.initialize(parameters);

            while (!imu.isGyroCalibrated()) {
                sleep(50);
            }
        } else
            { setStatus("IMU not found"); }
    }

    private void initMainLift() {
        setStatus("Initialising main lift mechanism");

        // Motor
        mainLift = hwMap.get(DcMotor.class, "main_lift");

        // Switches
        mainLiftUpperLimit = hwMap.get(DigitalChannel.class, "ml_upperlimit");
        mainLiftLowerLimit = hwMap.get(DigitalChannel.class, "ml_lowerlimit");

        mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainLift.setPower(0);

        mainLiftUpperLimit.setMode(DigitalChannel.Mode.INPUT);
        mainLiftLowerLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initDrive() {
        setStatus("Initialising drive base");
        // Motors
        rfDrive = hwMap.get(DcMotor.class, "rf_drive");
        rrDrive = hwMap.get(DcMotor.class, "rr_drive");
        lfDrive = hwMap.get(DcMotor.class, "lf_drive");
        lrDrive = hwMap.get(DcMotor.class, "lr_drive");

        setDriveDirection(DriveDirection.FORWARD);
        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveServo(double power) {
        double p = Math.abs(power);
        rfDrive.setPower(p);
        rrDrive.setPower(p);
        lfDrive.setPower(p);
        lrDrive.setPower(p);
    }

    private void setDrivePower(double power) {
        double p = Math.abs(power);
        rfDrive.setPower(p);
        rrDrive.setPower(p);
        lfDrive.setPower(p);
        lrDrive.setPower(p);
    }

    private void setDriveDirection(DriveDirection direction) {
        switch (direction) {
            case FORWARD:
                rfDrive.setDirection(FORWARD);
                rrDrive.setDirection(FORWARD);
                lfDrive.setDirection(REVERSE);
                lrDrive.setDirection(REVERSE);
                break;
            case REVERSE:
                rfDrive.setDirection(REVERSE);
                rrDrive.setDirection(REVERSE);
                lfDrive.setDirection(FORWARD);
                lrDrive.setDirection(FORWARD);
                break;
            case STRAFE_RIGHT:
                rfDrive.setDirection(REVERSE);
                rrDrive.setDirection(FORWARD);
                lfDrive.setDirection(REVERSE);
                lrDrive.setDirection(FORWARD);
                break;
            case STRAFE_LEFT:
                rfDrive.setDirection(FORWARD);
                rrDrive.setDirection(REVERSE);
                lfDrive.setDirection(FORWARD);
                lrDrive.setDirection(REVERSE);
                break;
            case TANK_CLOCKWISE:
                rfDrive.setDirection(REVERSE);
                rrDrive.setDirection(REVERSE);
                lfDrive.setDirection(REVERSE);
                lrDrive.setDirection(REVERSE);
                break;
            case TANK_ANTICLOCKWISE:
                rfDrive.setDirection(FORWARD);
                rrDrive.setDirection(FORWARD);
                lfDrive.setDirection(FORWARD);
                lrDrive.setDirection(FORWARD);
                break;
        }
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        rfDrive.setMode(mode);
        rrDrive.setMode(mode);
        lfDrive.setMode(mode);
        lrDrive.setMode(mode);
    }

    private void setStatus(String message) {
        if(DEBUG_LEVELS.INFO.ordinal() >= DEBUG.ordinal()) {    // only send status messages if debug level INFO or above
            this.teleStatus.setValue(message);
            this.telemetry.update();
        }
    }
}
