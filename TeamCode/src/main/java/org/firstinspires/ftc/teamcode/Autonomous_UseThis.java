/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.MalGlobals.DEBUG;

@TeleOp(name="Autononous_UseThis", group="Iterative Opmode")
public class Autonomous_UseThis extends OpMode implements MalBase
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // robot variables
    MalFunctionBot robot    = new MalFunctionBot();
    MalGlobals.DEBUG_LEVELS INFO = MalGlobals.DEBUG_LEVELS.INFO;
    MalGlobals.DEBUG_LEVELS VERBOSE = MalGlobals.DEBUG_LEVELS.VERBOSE;

    HashMap<String, Telemetry.Item> telemetryItems = new HashMap<String, Telemetry.Item>();

    int rot = 0;

    TargetDetection detection = new TargetDetection();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        this.telemetryItems.put("status", telemetry.addData("status", ""));
        robot.init(hardwareMap, telemetryItems.get("status"), telemetry, this);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // start sensing
        detection.startSensing();
        // start moving
        while (!detection.skystoneVisible){
            robot.drive(MalFunctionBot.DriveDirection.FORWARD, 1);
        }
        detection.positionAtTarget();
        robot.moveServo1(MalFunctionBot.ServoDirection.FULL_DOWN);
        runtime.reset();
        while (runtime.seconds() < 1.5){
            robot.drive(MalFunctionBot.DriveDirection.STRAFE_RIGHT, 1);
        }
        robot.drive(MalFunctionBot.DriveDirection.FORWARD, 1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void log(Object value, String itemName, MalGlobals.DEBUG_LEVELS level) {
        if (level.ordinal() >= DEBUG.ordinal()) {
            Telemetry.Item ti = this.telemetryItems.get(itemName);
            if(ti != null) {
                ti.setValue(value);
                telemetry.update();
            }
        }
    }
}