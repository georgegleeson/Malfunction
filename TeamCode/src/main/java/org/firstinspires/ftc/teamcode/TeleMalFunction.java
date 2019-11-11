package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.MalFunctionBot.*;
import static org.firstinspires.ftc.teamcode.MalGlobals.*;

@TeleOp(name="Mal Function Interactive", group="Interative Opmode")
public class TeleMalFunction extends OpMode implements MalBase {

    // get robot and set up variables
    MalFunctionBot robot    = new MalFunctionBot();
    DEBUG_LEVELS INFO = DEBUG_LEVELS.INFO;
    DEBUG_LEVELS VERBOSE = DEBUG_LEVELS.VERBOSE;


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
    }

    @Override
    public void loop() {
        // gamepad 1
        double stick1_lx = gamepad1.left_stick_x;    // left stick X
        double stick1_ly = gamepad1.left_stick_y;    // left stick Y
        double stick1_rx = gamepad1.right_stick_x;   // right stick X
        double stick1_ry = gamepad1.right_stick_y;   // right stick Y
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;
        boolean a1 = gamepad1.a;
        boolean b1 = gamepad1.b;

        // gamepad 2
        double stick2_lx = gamepad2.left_stick_x;   // gamepad 2 left stick X
        double stick2_ly = gamepad2.left_stick_y;   // gamepad 2 left stick Y
        double stick2_rx = gamepad2.right_stick_x;   // gamepad 2 right stick X
        double stick2_ry = gamepad2.right_stick_y;   // gamepad 2 right stick Y
        boolean x2 = gamepad2.x;
        boolean y2 = gamepad2.y;
        boolean a2 = gamepad2.a;
        boolean b2 = gamepad2.b;
        boolean bump2_l = gamepad2.left_bumper;
        boolean bump2_r = gamepad2.right_bumper;
        double trig2_l = gamepad2.left_trigger;
        double trig2_r = gamepad2.right_trigger;


        boolean l_trigger;

        int serv1 = 0;
        int serv2 = 0;

        // Boolean limit_up = robot.mainLiftUpperLimitState();     // scissor upper limit switch
        // Boolean limit_down = robot.mainLiftLowerLimitState();   // scissor lower limit switch

        if (gamepad1.left_trigger > 0) {
            l_trigger = true;
        } else {
            l_trigger = false;
        }

        //if (l_trigger){
        //    stick_ly = gamepad1.left_stick_y / 2;
        //    stick_rx = gamepad1.right_stick_x / 2;
        //    stick_lx = gamepad1.left_stick_x / 2;
        //    stick2_lx = gamepad2.left_stick_x / 2;
        //    stick2_rx = gamepad2.right_stick_x / 2;
        //} else {

        stick2_lx = gamepad2.left_stick_x;
        stick2_ly = gamepad2.left_stick_y;
        stick2_rx = gamepad2.right_stick_x;
        stick2_ry = gamepad2.right_stick_y;
        //}

        // servo moving

        // setting variables

        if (gamepad1.right_bumper || (gamepad1.right_trigger > 0)) {
            x1 = false;
            y1 = false;
        } else if (gamepad1.x || gamepad1.y) {
            x1 = true;
            y1 = true;
        }

        // move servo 1
        if (!x1 && !y1) {
            robot.moveServo1(ServoDirection.STOP);
        } else if (gamepad1.x) {
            robot.moveServo1(ServoDirection.FULL_DOWN);
        } else if (gamepad1.y) {
            robot.moveServo1(ServoDirection.FULL_UP);
        }

        // setting variables

        if (gamepad1.left_bumper || (gamepad1.left_trigger > 0)) {
            a1 = false;
            b1 = false;
        } else if (gamepad1.a || gamepad1.b) {
            a1 = true;
            b1 = true;
        }

        // move servo 2

        if (!a1 && !b1) {
            robot.moveServo2(ServoDirection.STOP);
        } else if (gamepad1.a) {
            robot.moveServo2(ServoDirection.FULL_DOWN);
        } else if (gamepad1.b) {
            robot.moveServo2(ServoDirection.FULL_UP);
        }

        // drive base

        log(stick1_ly, "dr_stick_ly", VERBOSE);
        log(stick1_rx, "dr_stick_lx", VERBOSE);

        // drive
        if ((stick1_lx + stick1_ly + stick1_rx) != 0) {
            if (stick1_ly > 0) {
                log("Mal is driving forward", "status", INFO);
                robot.drive(DriveDirection.FORWARD, stick1_ly);
            } else if (stick1_ly < 0) {
                log("Mal is driving backwards", "status", INFO);
                robot.drive(DriveDirection.REVERSE, stick1_ly);
            }

            // drive left and right
            if (stick1_lx > 0) {
                log("Mal is strafing right", "status", INFO);
                robot.drive(DriveDirection.STRAFE_RIGHT, stick1_lx);
            } else if (stick1_lx < 0) {
                log("Mal is strafing left", "status", INFO);
                robot.drive(DriveDirection.STRAFE_LEFT, stick1_lx);
            }

            // tank turn!
            if (stick1_rx > 0) {
                log("Mal is turning clockwise", "status", INFO);
                robot.drive(DriveDirection.TANK_CLOCKWISE, stick1_rx);
            } else if (stick1_rx < 0) {
                log("Mal is turning anticlockwise", "status", INFO);
                robot.drive(DriveDirection.TANK_ANTICLOCKWISE, stick1_rx);
            }
        } else {
            robot.drive(DriveDirection.STOP, 0);
        }

        // scissor lift control
        if (stick2_ly != 0) {
            if (stick2_ly > 0) {
                log("Mal is dropping the scissor", "status", INFO);
                robot.moveMainLift(LiftDirection.DOWN);
            } else if (stick2_ly < 0) {
                log("Mal is lifting the scissor", "status", INFO);
                robot.moveMainLift(LiftDirection.UP);
            }
            dpadReady = true;
//        } else if (gamepad2.dpad_up || gamepad2.dpad_down) {
//
//            if (gamepad2.dpad_up && dpadReady && scissorHeight < 5){
//                scissorHeight++;
//                dpadReady = false;
//            } else if (gamepad2.dpad_down && dpadReady && scissorHeight > 0){
//                scissorHeight--;
//                dpadReady = false;
//            }
//
//            // auto height movement
//            switch (scissorHeight){
//                case 0:
//                    robot.setLiftHeight(LiftPosition.FULL_DOWN);
//                    break;
//                case 1:
//                    robot.setLiftHeight(LiftPosition.BLOCK1);
//                    break;
//                case 2:
//                    robot.setLiftHeight(LiftPosition.BLOCK2);
//                    break;
//                case 3:
//                    robot.setLiftHeight(LiftPosition.BLOCK3);
//                    break;
//                case 4:
//                    robot.setLiftHeight(LiftPosition.BLOCK4);
//                    break;
//                case 5:
//                    robot.setLiftHeight(LiftPosition.FULL_UP);
//                    break;
//            }
        } else if (!robot.mainLiftMotorIsBusy()){
        robot.moveMainLift(LiftDirection.STOP);
    }

        // move 8020

        if(stick2_ry != 0 || trig2_r != 0) {
            robot.setArmMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (stick2_ry > 0) {
                robot.moveArm(ArmDirection.OUT);
            } else if(stick2_ry < 0) {
                robot.moveArm(ArmDirection.IN);
            } else if (trig2_r > 0) {
                robot.setArmPosition(ArmPosition.HOME);
            }
        } else {
            robot.moveArm(ArmDirection.STOP);
        }

        // move rotator

        if(gamepad2.right_stick_x != 0) {
            robot.setSpinnyMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.right_stick_x > 0) {
                robot.spinner(SpinnerRotationDirection.CLOCKWISE);
            } else if(gamepad2.right_stick_x < 0) {
                robot.spinner(SpinnerRotationDirection.ANTICLOCKWISE);
            }
        } else { robot.spinner(SpinnerRotationDirection.STOP);}

        // move gripper

        if(gamepad2.dpad_left || gamepad2.dpad_right) {
            if (gamepad2.dpad_left) {
                robot.gripper(GripperDirection.GRIP);
            } else if (gamepad2.dpad_right) {
                robot.gripper(GripperDirection.RELEASE);
            }
        } else if (gamepad2.left_trigger > 0){
            robot.gripper(GripperDirection.GRIP);
        } else { robot.gripper(GripperDirection.STOP);}

        // if(stick2_lx != 0 || stick2_ry != 0){
        // robot.moveServo(stick2_lx, stick2_ry);
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
