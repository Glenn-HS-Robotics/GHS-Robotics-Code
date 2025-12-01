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

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit


@Suppress("unused")
@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
class BasicOmniOpMode_Linear : LinearOpMode() {
//    // Declare OpMode members for each of the 4 motors.
//    private val runtime = ElapsedTime()
//    private var frontLeftDrive: DcMotor? = null
//    private var backLeftDrive: DcMotor? = null
//    private var frontRightDrive: DcMotor? = null
//    private var backRightDrive: DcMotor? = null
//
//    private var launchMoto: DcMotor? = null
//    private var intakeRuns:DcMotor? = null
//
//    // Servo + kicker state (only part changed) -----------------------------
//    private var pusher: Servo? = null     // servo on Expansion Hub 2, port 0, name: "servo_motor"
//    private val REST_POS = 1.0
//    private val KICK_POS = 0.5
//    private val KICK_TIME_MS = 1500
//
//    private enum class KickState { IDLE, EXTENDING, RETRACTING }
//    private var kickState = KickState.IDLE
//    private val phaseTimer = ElapsedTime()
//    // ----------------------------------------------------------------
//
//    private val LAUNCHER_POWER = 1.0
//    private val intake_power = 0.65
//
//    override fun runOpMode() {
//        // Initialize the hardware variables.
//
//        frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
//        backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
//        frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
//        backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")
//        intakeRuns = hardwareMap.get(DcMotor::class.java, "intake_motor")
//        launchMoto = hardwareMap.get(DcMotor::class.java, "launcher")
//        pusher = hardwareMap.get(Servo::class.java, "servo_motor")
//        pusher!!.position = REST_POS
//
//        frontLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
//        backLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
//        frontRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
//        backRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
//        launchMoto!!.direction = DcMotorSimple.Direction.REVERSE
//        intakeRuns!!.direction = DcMotorSimple.Direction.REVERSE
//
//        telemetry.addData("Status", "Initialized")
//        telemetry.update()
//
//        waitForStart()
//        runtime.reset()
//
//        var launcherEnabled = false
//        var prevB = false
//        var intakeEnabled = false
//        var prevA = false
//        var prevX = false          // added for X button edge-detect (servo only)
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            var max: Double
//
//            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//            val axial = -gamepad1.left_stick_y.toDouble() // Note: pushing stick forward gives negative value
//            val lateral = gamepad1.left_stick_x.toDouble()
//            val yaw = gamepad1.right_stick_x.toDouble()
//
//            val bNow = gamepad2.b
//            if(bNow && !prevB){
//                launcherEnabled = !launcherEnabled
//            }
//            prevB = bNow
//            val aNow = gamepad2.a
//            if(aNow && !prevA){
//                intakeEnabled = !intakeEnabled
//            }
//            prevA = aNow
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.
//            var frontLeftPower = axial + lateral + yaw
//            var frontRightPower = axial - lateral - yaw
//            var backLeftPower = axial - lateral + yaw
//            var backRightPower = axial + lateral - yaw
//
//            // Normalize the values so no wheel power exceeds 100% (This basically makes it so that nothing goes above 100)
//            // This ensures that the robot maintains the desired motion.
//            max = max(abs(frontLeftPower), abs(frontRightPower))
//            max = max(max, abs(backLeftPower))
//            max = max(max, abs(backRightPower))
//
//            if (max > 1.0) {
//                frontLeftPower /= max
//                frontRightPower /= max
//                backLeftPower /= max
//                backRightPower /= max
//            }
//
//            // Send calculated power to wheels
//            frontLeftDrive!!.power = frontLeftPower
//            frontRightDrive!!.power = frontRightPower
//            backLeftDrive!!.power = backLeftPower
//            backRightDrive!!.power = backRightPower
//
//            // Send Power to the Launcher
//            launchMoto!!.power = if(launcherEnabled) LAUNCHER_POWER else 0.0
//            intakeRuns!!.power = if(intakeEnabled) intake_power else 0.0
//
//            // Pressing x makes the servo trigger and return to original position
//            val xNow = gamepad2.x
//
//            // On rising edge of X and only if not in a kick, start a cycle
//            if (xNow && !prevX && kickState == KickState.IDLE) {
//                pusher!!.position = KICK_POS
//                phaseTimer.reset()
//                kickState = KickState.EXTENDING
//            }
//            prevX = xNow
//
//            when (kickState) {
//                KickState.IDLE -> {
//                    // ensure that the servo stays parked at rest
//                    pusher!!.position = REST_POS
//                }
//                KickState.EXTENDING -> {
//                    if (phaseTimer.milliseconds() >= KICK_TIME_MS) {
//                        pusher!!.position = REST_POS
//                        phaseTimer.reset()
//                        kickState = KickState.RETRACTING
//                    }
//                }
//                KickState.RETRACTING -> {
//                    // short settle; you can tweak 80.0 if needed
//                    if (phaseTimer.milliseconds() >= 100.0) {
//                        kickState = KickState.IDLE
//                    }
//                }
//            }
//            // ---------------------------------------------------------
//
//            // Telemetry gives all of the data about the hardware, etc along with power
//            telemetry.addData("Info", "Run Time: $runtime")
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower)
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower)
//            telemetry.addData("Launcher", "%4.2f (%s)", LAUNCHER_POWER, if (launcherEnabled) "ON" else "OFF")
//            telemetry.addData("Intake", "%4.2f (%s)", intake_power, if (intakeEnabled) "ON" else "OFF")
//            telemetry.addData("Pusher", "state=%s pos=%.2f", kickState, pusher!!.position)
//            telemetry.update()
//        }
//    }
    private  val READ_PERIOD: Int = 1

    private var huskyLens: HuskyLens? = null

    override fun runOpMode() {
        huskyLens = hardwareMap.get<HuskyLens?>(HuskyLens::class.java, "Webcam 1")

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        val rateLimit = Deadline(READ_PERIOD.toLong(), TimeUnit.SECONDS)

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire()

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens!!.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens!!.getDeviceName())
        } else {
            telemetry.addData(">>", "Press start to continue")
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens!!.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION)

        telemetry.update()
        waitForStart()

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue
            }
            rateLimit.reset()

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            val blocks = huskyLens!!.blocks()
            telemetry.addData("Block count", blocks.size)
            for (i in blocks.indices) {
                telemetry.addData("Block", blocks[i].toString())
                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 *
                 * These values have Java type int (integer).
                 */
            }

            telemetry.update()
        }
    }
}
