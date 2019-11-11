package org.firstinspires.ftc.teamcode;

import  com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

    // Grabby armMotor
    private DcMotor gripperMotor    = null;     // grabs the blocks
    private DcMotor spinnyMotor     = null;     // spins the blocks
    private DcMotor armMotor        = null;     // extends the blocks

    // Servos
    // TODO: Name these variables
    private Servo servo                 = null;
    private Servo servo2                = null;
    private ServoController servcont    = null;

    // Switches
    private DigitalChannel armZeroPosition;
    private DigitalChannel mainLiftZeroPosition;

    // REV IMU
    public BNO055IMU imu   = null;

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

    // OpMode object
    private OpMode opMode = null;

    // elapsed time counter for run to position commands
    private ElapsedTime runtime = new ElapsedTime();

    // shortcuts for enumerations
    static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;

    public boolean getArmZeroPosition() { return armZeroPosition.getState(); }
    public boolean isMainLiftHome() { return !mainLiftZeroPosition.getState(); }
    public boolean mainLiftMotorIsBusy(){ return mainLift.isBusy(); }

    // all possible drive operations
    public enum DriveDirection {
        FORWARD,
        REVERSE,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TANK_CLOCKWISE,
        TANK_ANTICLOCKWISE,
        STOP
    }

    // scissor lift directions
    public enum LiftDirection {
        UP,
        DOWN,
        STOP
    }

    // extendable armMotor directions
    public enum ArmDirection {
        IN,
        OUT,
        STOP
    }

    // block spinner directions
    public enum SpinnerRotationDirection{
        CLOCKWISE,
        ANTICLOCKWISE,
        STOP
    }

    // scissor lift stop positions
    public enum LiftPosition{
        BLOCK1,
        BLOCK2,
        BLOCK3,
        BLOCK4,
        FULL_UP,
        FULL_DOWN,
        HOME
    }

    // Arm stop positions
    public enum ArmPosition {
        FULL_IN,
        FULL_OUT,
        HOME
    }

    // Block gripperMotor directions
    public enum GripperDirection{
        GRIP,
        RELEASE,
        STOP
    }

    // servo movement possibilities
    public enum ServoDirection{
        UP,
        DOWN,
        FULL_UP,
        FULL_DOWN,
        STOP
    }

    public MalFunctionBot() {
        // turn on or off hardware modules
        robotConfig.put("IMU", false);
        robotConfig.put("drive", true);
        robotConfig.put("scissorLift", true);
        robotConfig.put("grippy", true);
        robotConfig.put("servos", true);
    }

    //
    public void init(HardwareMap ahwMap, Telemetry.Item ti, Telemetry telemetry, OpMode om) {
        this.teleStatus = ti;
        this.telemetry = telemetry;
        this.opMode = om;
        setStatus("Mal is waking up");

        hwMap = ahwMap;

        // run init functions for hardware modules if enabled
        // try {
            // ensure the order is set
            if (robotConfig.get("drive")) initDrive();
            if (robotConfig.get("scissorLift")) initMainLift();
            if (robotConfig.get("IMU")) initIMU();
            if (robotConfig.get("grippy")) initGrabber();
            if (robotConfig.get("servos")) initServos();

            setStatus("Mal is ready to rock");
            this.initComplete = true;
        // } catch (Exception e) {
            // this.message = e.toString();
            // throw e;
        // }
    }

    // initialisations

    private void initIMU() {
        if (robotConfig.get("IMU")) {
            setStatus("Initialising IMU");

            // set IMU units
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hwMap.get(BNO055IMU.class, "imu");

            // calibrate IMU
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
        // checks if drive module is initialised to prevent null refrence error
        if (robotConfig.get("drive")) {
            setStatus("Initialising drive base");
            // Motors
            rfDrive = hwMap.get(DcMotor.class, "rf_drive");
            rrDrive = hwMap.get(DcMotor.class, "rr_drive");
            lfDrive = hwMap.get(DcMotor.class, "lf_drive");
            lrDrive = hwMap.get(DcMotor.class, "lr_drive");

            // check the motors initialised properly
            if (rfDrive == null || rrDrive == null || lfDrive == null || lrDrive == null) {
                throw new NullPointerException("Unable to find drive motors");
            }

            // set motors ready to run
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
            mainLiftZeroPosition = hwMap.get(DigitalChannel.class, "ml_zeroswitch");

            // error checking
            if (mainLift == null || mainLiftZeroPosition == null) {
                throw new NullPointerException("Unable to find main lift components");
            }

            // setup scissor lift components
            mainLiftZeroPosition.setMode(DigitalChannel.Mode.INPUT);
            mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // bring scissor lift down to zero position for robot start
            // while(!this.isMainLiftHome()) {
            //     mainLift.setDirection(FORWARD);
            //     mainLift.setPower(0.5);
        // }

            // stop scissor lift and set encoder value to zero
            mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        } else {
            setStatus("Lift module not enabled in robot");
        }
    }

    private void initGrabber() {
        if (robotConfig.get("grippy")) {
            setStatus("Initialising grabby armMotor");

            // Motors
            gripperMotor = hwMap.get(DcMotor.class, "gripperMotor");
            spinnyMotor = hwMap.get(DcMotor.class, "spinnyMotor");
            armMotor = hwMap.get(DcMotor.class, "inout");

            // Switches
            armZeroPosition = hwMap.get(DigitalChannel.class, "arm_zeroswitch");

            // error checking
            if (gripperMotor == null || spinnyMotor == null || armMotor == null) {
                throw new NullPointerException("Unable to find grabber motors");
            }

            // configure motors
            gripperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            gripperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            spinnyMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            spinnyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // switch to input
            armZeroPosition.setMode(DigitalChannel.Mode.INPUT);

            // bring armMotor in to zero position for robot start
//            while(!getArmZeroPosition()) {
//                armMotor.setDirection(FORWARD);
//                armMotor.setPower(0.5);
//                telemetry.addData("arm", getArmZeroPosition());
//                telemetry.update();
//            }
//            setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            setStatus("Grabber module not enabled in robot");
        }
    }

    private void initServos() {
        // TODO: set names correctly and comment code

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

    // scissor lift control
    public void moveMainLift(LiftDirection direction) {
        if (robotConfig.get("scissorLift")) {
            // set motor to run without encoder
            mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // checks which enumeration value has been passed in
            switch (direction) {
                // if up then make it go up
                case UP:
                    // sets the required direction and power
                    mainLift.setDirection(REVERSE);
                    mainLift.setPower(1);
                    telemetry.addLine("scissor going up");
                    break;
                 // if down then make it go down
                case DOWN:
                    if(!isMainLiftHome()) {
                        telemetry.addLine("scissor going down");
                        mainLift.setDirection(FORWARD);
                        mainLift.setPower(1);
                    } else {

                    }
                    break;
                default:
                    mainLift.setPower(0);
            }
            telemetry.update();

        } else {
            setStatus("Lift module not enabled in robot");
        }
    }

    public void gripper(GripperDirection direction){
        if (robotConfig.get("grippy")) {
            switch (direction) {
                case GRIP:
                    if (true == true) { // !mainLiftUpperLimitState()
                        gripperMotor.setDirection(REVERSE);
                        gripperMotor.setPower(0.2);
                    } else mainLift.setPower(0);
                    break;
                case RELEASE:
                    if (true == true) { // !mainLiftLowerLimitState()
                        gripperMotor.setDirection(FORWARD);
                        gripperMotor.setPower(0.3);
                    } else mainLift.setPower(0);
                    break;
                default:
                    mainLift.setPower(0);
            }
        } else {
            setStatus("Gripper module not enabled in robot");
        }
    }

    public void spinner(SpinnerRotationDirection direction){
        if (robotConfig.get("grippy")) {
            switch (direction) {
                case CLOCKWISE:
                    if (true == true) { // !mainLiftUpperLimitState()
                        spinnyMotor.setDirection(FORWARD);
                        spinnyMotor.setPower(0.25);
                    } else spinnyMotor.setPower(0);
                    break;
                case ANTICLOCKWISE:
                    if (true == true) { // !mainLiftLowerLimitState()
                        spinnyMotor.setDirection(REVERSE);
                        spinnyMotor.setPower(0.25);
                    } else spinnyMotor.setPower(0);
                    break;
                default:
                    spinnyMotor.setPower(0);
            }
        } else {
            setStatus("Gripper module not enabled in robot");
        }
    }

    public void moveArm(ArmDirection direction){
        if (robotConfig.get("grippy")) {
            switch (direction) {
                case OUT:
                    armMotor.setDirection(FORWARD);
                    armMotor.setPower(1);
                    break;
                case IN:
                    // if (!getArmZeroPosition()){
                        armMotor.setDirection(REVERSE);
                        armMotor.setPower(1);
                    // } else {
                    //    setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // }
                    break;
                default:
                    armMotor.setPower(0);
            }
        } else {
            setStatus("Gripper module not enabled in robot");
        }
    }

    // function to set the mode of the motors
    public void setArmMode(DcMotor.RunMode mode){
        armMotor.setMode(mode);
    }

    public void setSpinnyMode(DcMotor.RunMode mode){
        spinnyMotor.setMode(mode);
    }

    public void drive(DriveDirection direction, double power) {
        if (robotConfig.get("drive")) {
            // checks if the bot is going to strafe and if so calls the strafe drive function, if not calls the regular drive function
            setDriveDirection(direction);
            if (direction == DriveDirection.STRAFE_RIGHT || direction == DriveDirection.STRAFE_LEFT){
                setDriveStrafePower(power);
            } else {
                setDrivePower(power);
            }
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    public void moveServo1(ServoDirection direction){
        if (robotConfig.get("servos")) {
            // sets servo positions based on enumeration values
            switch (direction) {
                case UP:
                    // makes the servo go slow by setting position to be at slight increments each loop through
                    servo.setPosition(servo.getPosition() + 0.01);
                    break;
                case DOWN:
                    servo.setPosition(servo.getPosition() - 0.01);
                    break;
                case FULL_UP:
                    // sets full positions
                    servo.setPosition(1);
                    break;
                case FULL_DOWN:
                    servo.setPosition(-1);
                    break;
                default:
                    servo.setPosition(servo.getPosition());
            }
        } else {
            setStatus("Servo module not enabled in robot");
        }
    }

    public void moveServo2(ServoDirection direction) {
        if (robotConfig.get("servos")) {
            switch (direction) {
                case UP:
                    servo2.setPosition(servo2.getPosition() + 0.1);
                    break;
                case DOWN:
                    servo2.setPosition(servo2.getPosition() - 0.1);
                    break;
                case FULL_UP:
                    servo2.setPosition(1);
                    break;
                case FULL_DOWN:
                    servo2.setPosition(-1);
                    break;
                default:
                    servo2.setPosition(servo2.getPosition());
            }
        } else {
            setStatus("Servo module not enabled in robot");
        }
    }

    private void setDrivePower(double power) {
        if (robotConfig.get("drive")) {
            // sets all motors to equal the same power
            // the power is taken as an absolute value as negatives are not required because the motor direction is changing in the move main lift function
            double p = Math.abs(power);
            rfDrive.setPower(p);
            rrDrive.setPower(p);
            lfDrive.setPower(p);
            lrDrive.setPower(p);
        } else {
            setStatus("Drive module not enabled in robot");
        }
    }

    private void setDriveStrafePower(double power) {
        if (robotConfig.get("drive")) {
            double p = Math.abs(power);
            // multiply motor powers by constants to lower their powers for strafing to account for weight differences whilst maintaining variable speed
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
                    // sets motor directions to go in the directions required to move in the way specified
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

    public void setArmPosition(ArmPosition height) {
        armMotor.setDirection(FORWARD);

        if (!getArmZeroPosition()){
            switch (height) {
                case FULL_IN:
                    armMotor.setTargetPosition(0);
                    break;
                case FULL_OUT:
                    armMotor.setTargetPosition(50);
                    break;
                case HOME:
                    while (!getArmZeroPosition()){
                        moveArm(ArmDirection.IN);
                    }
                    setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                default:
                    armMotor.setTargetPosition(0);
            }

            runtime.reset();
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);

            // wait for armMotor to finish move
            while(armMotor.isBusy()) {
                // make sure that the limit switch is not activated such that the motor does not drive past where it should
                if (!getArmZeroPosition()){
                    // put encoder position to telemetry
                    telemetry.addData("Encoder pos", mainLift.getCurrentPosition());
                    setStatus("Arm is running to position");
                    sleep(1);
                } else {
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                }
            }
        } else {
            // else stop the motor and reset the encoder to zero
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setLiftHeight(LiftPosition height){
            int position = 0;
            // set int position to different values depending on desired end encoder position
            switch (height) {
                case BLOCK1:
                    position = 460; // mainLift.setTargetPosition(460);
                    break;
                case BLOCK2:
                    position = 920; // mainLift.setTargetPosition(920);
                    break;
                case BLOCK3:
                    position = 1380; // mainLift.setTargetPosition(1380);
                    break;
                case BLOCK4:
                    position = 1840; // mainLift.setTargetPosition(1840);
                    break;
                case FULL_DOWN:
                    position = 0; // mainLift.setTargetPosition(0);
                    break;
                case FULL_UP:
                    position = 2300; // mainLift.setTargetPosition(2300);
                    break;
                case HOME:
                    position = 0;
                    // while (mainLiftZeroPosition()) {
                    //     moveMainLift(LiftDirection.DOWN);
                    // }
                    // setLiftMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
            }

            // if the position isnt already the desired position, set int position to motor target position and set it to run to position
            if(mainLift.getCurrentPosition() != position) {
                mainLift.setTargetPosition(position);
                setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                mainLift.setPower(0.5);

                // wait for mainLift to finish move
                while(mainLift.isBusy() && !isMainLiftHome()) {
                    telemetry.addData("Encoder pos", mainLift.getCurrentPosition());
                    setStatus("Main Lift is running to position");
                }
            }

            // stop motor and reset mode
            mainLift.setPower(0);
            setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public float robotAngle(){
        // function that returns the z angle of the robot
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }

//    public void rotateDrive(float rot){
//        if (robotConfig.get("IMU") && robotConfig.get("drive")) {
//            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle != rot){
//                if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < rot) {
//                    if (rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
//                        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    setDriveDirection(DriveDirection.TANK_CLOCKWISE);
//                    setDrivePower(0.2);
//                } else if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > rot) {
//                    if (rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
//                        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    setDriveDirection(DriveDirection.TANK_ANTICLOCKWISE);
//                    setDrivePower(1);
//                } else {
//
//                }
//            }
//
//        } else {
//            setStatus("Drive or IMU module not enabled in robot");
//        }
//    }

    public void setDriveMode(DcMotor.RunMode mode) {
        if (robotConfig.get("drive")) {
            // set drive motors to desired mode
            rfDrive.setMode(mode);
            rrDrive.setMode(mode);
            lfDrive.setMode(mode);
            lrDrive.setMode(mode);
        }
    }

    private void setStatus(String message) {
        // check the status of the telemetry and update it accordingly
        if(DEBUG_LEVELS.INFO.ordinal() >= DEBUG.ordinal()) {    // only send status messages if debug level INFO or above
            this.teleStatus.setValue(message);
            this.telemetry.update();
        }
    }
}

