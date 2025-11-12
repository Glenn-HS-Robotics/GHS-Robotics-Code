/* Copyright (c) 2021 FIRST. All rights reserved.
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
package teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.max


/*
 * This file contains an example of a Linear "OpMode".
 */
@Suppress("unused")
@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
class BasicOmniOpMode_Linear : LinearOpMode() {
    // Declare OpMode members for each of the 4 motors.
    private val runtime = ElapsedTime()
    private var frontLeftDrive: DcMotor? = null
    private var backLeftDrive: DcMotor? = null
    private var frontRightDrive: DcMotor? = null
    private var backRightDrive: DcMotor? = null

    private var launchMoto: DcMotor? = null
    private var intakeRuns:DcMotor? = null

    // ==== Servo + kicker state (only part changed) ====
    private var pusher: Servo? = null     // servo on Expansion Hub 2, port 0, name: "servo_motor"
    private val REST_POS = 1.0
    private val KICK_POS = 0.5
    private val KICK_TIME_MS = 1500

    private enum class KickState { IDLE, EXTENDING, RETRACTING }
    private var kickState = KickState.IDLE
    private val phaseTimer = ElapsedTime()
    // ================================================

    private val LAUNCHER_POWER = 1.0
    private val intake_power = 0.65

    override fun runOpMode() {
        // Initialize the hardware variables.

        frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
        backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
        frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
        backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")
        intakeRuns = hardwareMap.get(DcMotor::class.java, "intake_motor")
        launchMoto = hardwareMap.get(DcMotor::class.java, "launcher")
        pusher = hardwareMap.get(Servo::class.java, "servo_motor")
        pusher!!.position = REST_POS

        frontLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
        launchMoto!!.direction = DcMotorSimple.Direction.REVERSE
        intakeRuns!!.direction = DcMotorSimple.Direction.REVERSE

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        var launcherEnabled = false
        var prevB = false
        var intakeEnabled = false
        var prevA = false
        var prevX = false          // <=== added for X button edge-detect (servo only)

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            var max: Double

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            val axial = -gamepad1.left_stick_y.toDouble() // Note: pushing stick forward gives negative value
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()

            val bNow = gamepad2.b
            if(bNow && !prevB){
                launcherEnabled = !launcherEnabled
            }
            prevB = bNow
            val aNow = gamepad2.a
            if(aNow && !prevA){
                intakeEnabled = !intakeEnabled
            }
            prevA = aNow
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            var frontLeftPower = axial + lateral + yaw
            var frontRightPower = axial - lateral - yaw
            var backLeftPower = axial - lateral + yaw
            var backRightPower = axial + lateral - yaw

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = max(abs(frontLeftPower), abs(frontRightPower))
            max = max(max, abs(backLeftPower))
            max = max(max, abs(backRightPower))

            if (max > 1.0) {
                frontLeftPower /= max
                frontRightPower /= max
                backLeftPower /= max
                backRightPower /= max
            }

            // Send calculated power to wheels
            frontLeftDrive!!.power = frontLeftPower
            frontRightDrive!!.power = frontRightPower
            backLeftDrive!!.power = backLeftPower
            backRightDrive!!.power = backRightPower

            // Send Power to the Launcher
            launchMoto!!.power = if(launcherEnabled) LAUNCHER_POWER else 0.0
            intakeRuns!!.power = if(intakeEnabled) intake_power else 0.0

            // ===== X-triggered single kick cycle (servo code only) =====
            val xNow = gamepad2.x

            // On rising edge of X and only if not in a kick, start a cycle
            if (xNow && !prevX && kickState == KickState.IDLE) {
                pusher!!.position = KICK_POS
                phaseTimer.reset()
                kickState = KickState.EXTENDING
            }
            prevX = xNow

            when (kickState) {
                KickState.IDLE -> {
                    // ensure we stay parked at rest
                    pusher!!.position = REST_POS
                }
                KickState.EXTENDING -> {
                    if (phaseTimer.milliseconds() >= KICK_TIME_MS) {
                        pusher!!.position = REST_POS
                        phaseTimer.reset()
                        kickState = KickState.RETRACTING
                    }
                }
                KickState.RETRACTING -> {
                    // short settle; you can tweak 80.0 if needed
                    if (phaseTimer.milliseconds() >= 100.0) {
                        kickState = KickState.IDLE
                    }
                }
            }
            // ===========================================================

            // Show the elapsed game time and wheel power.
            telemetry.addData("Info", "Run Time: $runtime")
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower)
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower)
            telemetry.addData("Launcher", "%4.2f (%s)", LAUNCHER_POWER, if (launcherEnabled) "ON" else "OFF")
            telemetry.addData("Intake", "%4.2f (%s)", intake_power, if (intakeEnabled) "ON" else "OFF")
            telemetry.addData("Pusher", "state=%s pos=%.2f", kickState, pusher!!.position)
            telemetry.update()
        }
    }
}
