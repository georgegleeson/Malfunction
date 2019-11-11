package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.MalFunctionBot.*;
import static org.firstinspires.ftc.teamcode.MalGlobals.*;

@Disabled
@TeleOp(name="Mal Function Interactive", group="Interative Opmode")
public class Revertable2 extends OpMode implements MalBase {

    MalFunctionBot robot    = new MalFunctionBot();
    DEBUG_LEVELS INFO = DEBUG_LEVELS.INFO;
    DEBUG_LEVELS VERBOSE = DEBUG_LEVELS.VERBOSE;

    HashMap<String, Telemetry.Item> telemetryItems = new HashMap<String, Telemetry.Item>();

    @Override
    public void init() {
        initTelemetry();
        // robot.init(hardwareMap, telemetryItems.get("status"), telemetry);
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

        double stick_ly;    // left stick Y
        double stick_lx;    // left stick X
        double stick_ry;    // right stick Y
        double stick_rx;    // right stick X
        boolean x1 = false;
        boolean y1 = false;
        boolean a1 = false;
        boolean b1 = false;

        //gamepad 2

        double stick2_lx;
        double stick2_rx;

        boolean l_trigger;

        int serv1 = 0;
        int serv2 = 0;

        // Boolean limit_up = robot.mainLiftUpperLimitState();     // scissor upper limit switch
        // Boolean limit_down = robot.mainLiftLowerLimitState();   // scissor lower limit switch

        if (gamepad1.left_trigger > 0){
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
        stick_ly = gamepad1.left_stick_y;
        stick_rx = gamepad1.right_stick_x;
        stick_lx = gamepad1.left_stick_x;
        stick2_lx = gamepad2.left_stick_x;
        stick2_rx = gamepad2.right_stick_y;
        //}

        // gamepad 1 - driving

        // servo moving

        if (gamepad1.right_bumper || (gamepad1.right_trigger > 0)){
            x1 = false;
            y1 = false;
        } else if (gamepad1.x || gamepad1.y){
            x1 = true;
            y1 = true;
        }

        if (gamepad1.right_bumper && !(gamepad1.right_trigger > 0)){
            serv1 = 1;
        } else if (!gamepad1.right_bumper && (gamepad1.right_trigger > 0)){
            serv1 = -1;
        } else if (!x1 && !y1) {
            serv1 = 0;
        } else if (gamepad1.x) {
            serv1 = -2;
        } else if (gamepad1.y) {
            serv1 = 2;
        }

        if (gamepad1.left_bumper || (gamepad1.left_trigger > 0)){
            a1 = false;
            b1 = false;
        } else if (gamepad1.a || gamepad1.b){
            a1 = true;
            b1 = true;
        }

        if (gamepad1.left_bumper && !(gamepad1.left_trigger > 0)){
            serv2 = 1;
        } else if (!gamepad1.left_bumper && (gamepad1.left_trigger > 0)){
            serv2 = -1;
        } else if (!a1 && !b1) {
            serv2 = 0;
        } else if (gamepad1.a) {
            serv2 = -2;
        } else if (gamepad1.b) {
            serv2 = 2;
        }

        // robot.moveServo(serv1, serv2);

        // drive base

        log(stick_ly, "dr_stick_ly", VERBOSE);

        // drive backwards and forwards

        if(stick_ly > 0) {
            log("Mal is driving forward", "status", INFO);
            robot.drive(DriveDirection.FORWARD, stick_ly);
        } else if(stick_ly < 0) {
            log("Mal is driving backwards", "status", INFO);
            robot.drive(DriveDirection.REVERSE, stick_ly);
        } else {
            log("Mal is stopped", "status", INFO);
            robot.drive(DriveDirection.STOP, 0);
        }

        // drive left and right

        if (stick_lx > 0) {
            log("Mal is strafing right", "status", INFO);
            robot.drive(DriveDirection.STRAFE_RIGHT, stick_lx);
        } else if (stick_lx < 0) {
            log("Mal is strafing left", "status", INFO);
            robot.drive(DriveDirection.STRAFE_LEFT, stick_lx);
        } else {
            log("Mal is stopped", "status", INFO);
            robot.drive(DriveDirection.STOP, 0);
        }

        // tank turn!

        log(stick_rx, "dr_stick_lx", VERBOSE);

        if (stick_rx > 0) {
            log("Mal is turning clockwise", "status", INFO);
            robot.drive(DriveDirection.TANK_CLOCKWISE, stick_rx);
        } else if (stick_rx < 0) {
            log("Mal is turning anticlockwise", "status", INFO);
            robot.drive(DriveDirection.TANK_ANTICLOCKWISE, stick_rx);
        } else {
            log("Mal is stopped", "status", INFO);
            robot.drive(DriveDirection.STOP, 0);
        }

        // gamepad 2 - lifting

        // scissor

        if(gamepad2.left_stick_y != 0) {
            robot.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.left_stick_y > 0) {
                log("Mal is lifting the scissor", "status", INFO);
                robot.moveMainLift(LiftDirection.UP);
            } else if(gamepad2.left_stick_y < 0) {
                log("Mal is dropping the scissor", "status", INFO);
                robot.moveMainLift(LiftDirection.DOWN);
            }
        /* } else if (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y || gamepad2.left_bumper || gamepad2.right_bumper){
            robot.setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.a){
                robot.setLiftHeight(1);
            } else if (gamepad2.b){
                robot.setLiftHeight(2);
            } else if (gamepad2.x){
                robot.setLiftHeight(3);
            } else if (gamepad2.y){
                robot.setLiftHeight(4);
            } else if (gamepad2.left_bumper){
                robot.setLiftHeight(5);
            } else if (gamepad2.right_bumper){
                robot.setLiftHeight(6);
            }
         */
        } else { robot.moveMainLift(LiftDirection.STOP);}

        // grippy arm

        //robot.grippy((gamepad2.left_trigger > 0), gamepad2.right_stick_x, gamepad2.right_stick_y);

        // if(stick2_lx != 0 || stick2_ry != 0){
        // robot.moveServo(stick2_lx, stick2_ry);
    }



    @Override
    public void stop() {
        super.stop();
    }

    public void log(Object value, String itemName, DEBUG_LEVELS level) {
        if (level.ordinal() >= DEBUG.ordinal()) {
            Telemetry.Item ti = this.telemetryItems.get(itemName);
            if(ti != null) {
                ti.setValue(value);
                telemetry.update();
            }
        }
    }

    private void initTelemetry() {
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
