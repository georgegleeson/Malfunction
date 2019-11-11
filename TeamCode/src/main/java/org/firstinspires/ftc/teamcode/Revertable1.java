package org.firstinspires.ftc.teamcode;

import  com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.MalGlobals.*;

public class Revertable1 {
    // Motors
    private DcMotor rfDrive  = null;     // Front Right
    private DcMotor rrDrive  = null;     // Rear Right
    private DcMotor lfDrive  = null;     // Front Left
    private DcMotor lrDrive  = null;     // Rear Left
    private DcMotor mainLift = null;     // scissor lift

    // Servos

    private Servo servo = null;
    private Servo servo2 = null;
    private ServoController servcont = null;

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

    // Grabby arm
    private DcMotor gripper  = null;
    private DcMotor spinny  = null;
    private DcMotor inout = null;

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

    public Revertable1() {
        robotConfig.put("IMU", true);
        robotConfig.put("drive", true);
        robotConfig.put("scissorLift", true);
        robotConfig.put("grippy", true);
        robotConfig.put("servos", false);
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
            if (robotConfig.get("grippy")) initGrabber();
            if (robotConfig.get("servos")) initServos();

            setStatus("Mal is ready to rock");
            this.initComplete = true;
        } catch (Exception e) {
            this.message = e.toString();
        }
    }

    // initialisations

    private void initIMU() {
        if (robotConfig.get("IMU")) {
            setStatus("Initialising IMU");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hwMap.get(BNO055IMU.class, "imu");

            if (imu != null) {
                imu.initialize(parameters);

                while (!imu.isGyroCalibrated()) {
                    sleep(50);
                }
            } else {
                setStatus("IMU not found");
            }
        } else {
            setStatus("IMU module not enabled in robot");
        }
    }

    private void initDrive() {
        if (robotConfig.get("drive")) {
            setStatus("Initialising drive base");
            // Motors
            rfDrive = hwMap.get(DcMotor.class, "rf_drive");
            rrDrive = hwMap.get(DcMotor.class, "rr_drive");
            lfDrive = hwMap.get(DcMotor.class, "lf_drive");
            lrDrive = hwMap.get(DcMotor.class, "lr_drive");

            if (rfDrive == null || rrDrive == null || lfDrive == null || lrDrive == null) {
                throw new NullPointerException("Unable to find drive motors");
            }

            setDriveDirection(DriveDirection.FORWARD);
            setDrivePower(0);
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    private void initMainLift() {
        if (robotConfig.get("scissorLift")) {
            setStatus("Initialising main lift mechanism");

            // Motor
            mainLift = hwMap.get(DcMotor.class, "main_lift");

            // Switches
            // mainLiftUpperLimit = hwMap.get(DigitalChannel.class, "ml_upperlimit");
            // fmainLiftLowerLimit = hwMap.get(DigitalChannel.class, "ml_lowerlimit");

            if (mainLift == null) {
                throw new NullPointerException("Unable to find main lift");
            }

            mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mainLift.setPower(0);

            // mainLiftUpperLimit.setMode(DigitalChannel.Mode.INPUT);
            // mainLiftLowerLimit.setMode(DigitalChannel.Mode.INPUT);
        } else {
            setStatus("Lift module not enabled in robot");
        }
    }

    private void initGrabber() {
        if (robotConfig.get("grippy")) {
            setStatus("Initialising grabby arm");
            // Motors
            gripper = hwMap.get(DcMotor.class, "gripper");
            gripper.setDirection(DcMotorSimple.Direction.FORWARD);
            spinny = hwMap.get(DcMotor.class, "spinny");
            spinny.setDirection(DcMotorSimple.Direction.FORWARD);
            inout = hwMap.get(DcMotor.class, "inout");
            inout.setDirection(DcMotorSimple.Direction.FORWARD);

            if (gripper == null || spinny == null || inout == null) {
                throw new NullPointerException("Unable to find grabber motors");
            }
        } else {
            setStatus("Grabber module not enabled in robot");
        }
    }

    private void initServos() {
        if (robotConfig.get("servos")){
            setStatus("Initialising servos");
            // Servos
            servo = hwMap.get(Servo.class, "servo");
            servo2 = hwMap.get(Servo.class, "servo2");
            servcont = servo.getController();
            servcont.pwmEnable();

            if (servo == null || servo2 == null || servcont == null) {
                throw new NullPointerException("Unable to find servo motors");
            }
        } else {
            setStatus("Servo module not enabled in robot");
        }
    }

    // public boolean mainLiftUpperLimitState() { return mainLiftUpperLimit.getState(); }
    // public boolean mainLiftLowerLimitState() { return mainLiftLowerLimit.getState(); }

    // doing stuff functions

    public void moveMainLift(LiftDirection direction) {
        if (robotConfig.get("scissorLift")) {
            switch (direction) {
                case UP:
                    if (true == true) { // !mainLiftUpperLimitState()
                        mainLift.setDirection(FORWARD);
                        mainLift.setPower(1);
                    } else mainLift.setPower(0);
                    break;
                case DOWN:
                    if (true == true) { // !mainLiftLowerLimitState()
                        mainLift.setDirection(REVERSE);
                        mainLift.setPower(1);
                    } else mainLift.setPower(0);
                    break;
                default:
                    mainLift.setPower(0);
            }
        } else {
            setStatus("Lift module not enabled in robot");
        }
    }

    public void grippy(double dir, double rot, double dis){
        if (robotConfig.get("grippy")) {
            gripper.setPower(dir / 4);
            spinny.setPower(rot / 4);
            inout.setPower(dis);
        } else {
            setStatus("Gripper module not enabled in robot");
        }
    }

    public void drive(DriveDirection direction, double power) {
        if (robotConfig.get("drive")) {
            setDriveDirection(direction);
            setDrivePower(power);
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    public void moveServo(int sv1, int sv2){
        if (robotConfig.get("servos")) {
            if (sv1 == 1) {
                servo.setPosition(servo.getPosition() + 0.1);
            } else if (sv1 == -1) {
                servo.setPosition(servo.getPosition() - 0.1);
            } else if (sv1 == 2) {
                servo.setPosition(1);
            } else if (sv1 == -2) {
                servo.setPosition(-1);
            }

            if (sv2 == 1) {
                servo2.setPosition(servo2.getPosition() + 0.1);
            } else if (sv2 == -1) {
                servo2.setPosition(servo2.getPosition() - 0.1);
            } else if (sv2 == 2) {
                servo2.setPosition(1);
            } else if (sv2 == -2) {
                servo2.setPosition(-1);
            }
        } else {
            setStatus("Servo module not enabled in robot");
        }
    }

    private void setDrivePower(double power) {
        if (robotConfig.get("drive")) {
            double p = Math.abs(power);
            rfDrive.setPower(p);
            rrDrive.setPower(p);
            lfDrive.setPower(p);
            lrDrive.setPower(p);
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    private void setDriveDirection(DriveDirection direction) {
        if (robotConfig.get("drive")) {
            switch (direction) {
                case STRAFE_LEFT:
                    rfDrive.setDirection(FORWARD);
                    rrDrive.setDirection(FORWARD);
                    lfDrive.setDirection(REVERSE);
                    lrDrive.setDirection(REVERSE);
                    break;
                case STRAFE_RIGHT:
                    rfDrive.setDirection(REVERSE);
                    rrDrive.setDirection(REVERSE);
                    lfDrive.setDirection(FORWARD);
                    lrDrive.setDirection(FORWARD);
                    break;
                case REVERSE:
                    rfDrive.setDirection(REVERSE);
                    rrDrive.setDirection(FORWARD);
                    lfDrive.setDirection(REVERSE);
                    lrDrive.setDirection(FORWARD);
                    break;
                case FORWARD:
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
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    /*
    public void autoMov(int dis, int rot){
        if (robotConfig.get("IMU") && robotConfig.get("drive")) {
            if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < rot) {
                if (rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                    setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                setDriveDirection(DriveDirection.TANK_CLOCKWISE);
                setDrivePower(0.2);
            } else if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > rot) {
                if (rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                    setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                setDriveDirection(DriveDirection.TANK_ANTICLOCKWISE);
                setDrivePower(1);
            } else {
                if (rfDrive.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                    setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (rfDrive.getCurrentPosition() != dis){
                    if (rfDrive.getCurrentPosition() < dis){
                        setDriveDirection(DriveDirection.REVERSE);
                        setDrivePower(0.5);
                    } else if (rfDrive.getCurrentPosition() > dis){
                        setDriveDirection(DriveDirection.FORWARD);
                        setDrivePower(0.5);
                    } else {
                        setDriveDirection(DriveDirection.FORWARD);
                        setDrivePower(0);
                    }
                }
            }
        } else {
            setStatus("Drive or IMU module not enabled in robot");
        }
    }
    */

    public void setLiftMode(DcMotor.RunMode mode){
        mainLift.setMode(mode);
    }

    public void setLiftHeight(int height){
        mainLift.setPower(0.5);
        switch (height){
            case 1:
                mainLift.setTargetPosition(20);
                break;
            case 2:
                mainLift.setTargetPosition(40);
                break;
            case 3:
                mainLift.setTargetPosition(60);
                break;
            case 4:
                mainLift.setTargetPosition(80);
                break;
            case 5:
                mainLift.setTargetPosition(0);
                break;
            case 6:
                mainLift.setTargetPosition(100);
                break;
            default:
                mainLift.setTargetPosition(0);
        }

        setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        if (robotConfig.get("drive")) {
            rfDrive.setMode(mode);
            rrDrive.setMode(mode);
            lfDrive.setMode(mode);
            lrDrive.setMode(mode);
        }
    }

    private void setStatus(String message) {
        if(DEBUG_LEVELS.INFO.ordinal() >= DEBUG.ordinal()) {    // only send status messages if debug level INFO or above
            this.teleStatus.setValue(message);
            this.telemetry.update();
        }
    }
}
