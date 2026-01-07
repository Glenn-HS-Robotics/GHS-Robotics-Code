/* Copyright (c) 2021 FIRST. All rights reserved. */
package teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max

@Suppress("unused")
@TeleOp(name = "Basic: Omni Linear OpMode Encoder", group = "Linear OpMode")
class BasicOmniOpModeEncoder_Linear : LinearOpMode() {

    private val runtime = ElapsedTime()
    private var frontLeftDrive: DcMotor? = null
    private var backLeftDrive: DcMotor? = null
    private var frontRightDrive: DcMotor? = null
    private var backRightDrive: DcMotor? = null

    private var launchMoto: DcMotor? = null
    private var intakeRuns:DcMotor? = null

    // Servo + kicker state
    private var pusher: Servo? = null
    private val REST_POS = 1.0
    private val KICK_POS = 0.3
    private val KICK_TIME_MS = 1500

    private enum class KickState { IDLE, EXTENDING, RETRACTING }
    private var kickState = KickState.IDLE
    private val phaseTimer = ElapsedTime()

    private val LAUNCHER_POWER = 1.0
    private val intake_power = 0.65

    override fun runOpMode() {

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
        intakeRuns!!.direction = DcMotorSimple.Direction.FORWARD

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        frontLeftDrive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRightDrive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeftDrive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRightDrive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeftDrive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frontRightDrive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backLeftDrive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backRightDrive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER

        var launcherEnabled = false
        var prevB = false
        var intakeEnabled = false
        var prevA = false
        var prevX = false

        while (opModeIsActive()) {

            this.telemetry.addLine("frontLeft" + frontLeftDrive!!.currentPosition)
            this.telemetry.addLine("frontRight" + frontRightDrive!!.currentPosition)
            this.telemetry.addLine("backLeft" + backLeftDrive!!.currentPosition)
            this.telemetry.addLine("backRight" + backRightDrive!!.currentPosition)

            var max: Double

            val axial = -gamepad1.left_stick_y.toDouble()
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

            var frontLeftPower = axial + lateral + yaw
            var frontRightPower = axial - lateral - yaw
            var backLeftPower = axial - lateral + yaw
            var backRightPower = axial + lateral - yaw

            max = max(abs(frontLeftPower), abs(frontRightPower))
            max = max(max, abs(backLeftPower))
            max = max(max, abs(backRightPower))

            if (max > 1.0) {
                frontLeftPower /= max
                frontRightPower /= max
                backLeftPower /= max
                backRightPower /= max
            }

            frontLeftDrive!!.power = frontLeftPower
            frontRightDrive!!.power = frontRightPower
            backLeftDrive!!.power = backLeftPower
            backRightDrive!!.power = backRightPower

            launchMoto!!.power = if(launcherEnabled) LAUNCHER_POWER else 0.0
            intakeRuns!!.power = if(intakeEnabled) intake_power else 0.0

            val xNow = gamepad2.x

            if (xNow && !prevX && kickState == KickState.IDLE) {
                pusher!!.position = KICK_POS
                phaseTimer.reset()
                kickState = KickState.EXTENDING
            }
            prevX = xNow

            // <<< ADDED: servo state machine for timed retract
            when (kickState) {
                KickState.EXTENDING -> {
                    if (phaseTimer.milliseconds() >= KICK_TIME_MS.toDouble()) {
                        pusher!!.position = REST_POS       // <<< ADDED
                        kickState = KickState.IDLE          // <<< ADDED
                    }
                }
                else -> {}
            }
            // <<< END ADDED SECTION

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