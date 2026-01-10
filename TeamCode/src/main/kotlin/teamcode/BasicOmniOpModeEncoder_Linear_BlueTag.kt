/* Copyright (c) 2021 FIRST. All rights reserved. */
package teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign

@Suppress("unused")
@TeleOp(name = "Basic: Omni Linear OpMode Encoder BLUE Tag", group = "Linear OpMode")
class BasicOmniOpModeEncoder_Linear_BlueTag : LinearOpMode() {

    private val runtime = ElapsedTime()
    private var frontLeftDrive: DcMotor? = null
    private var backLeftDrive: DcMotor? = null
    private var frontRightDrive: DcMotor? = null
    private var backRightDrive: DcMotor? = null

    private var launchMoto: DcMotor? = null
    private var intakeRuns: DcMotor? = null

    private var hoodServoLeft: Servo? = null
    private var hoodServoRight: Servo? = null



    // Servo + kicker state
    private var pusher: Servo? = null
    private val REST_POS = 1.0
    private val KICK_POS = 0.3
    private val KICK_TIME_MS = 450   // faster return

    private enum class KickState { IDLE, EXTENDING, RETRACTING }
    private var kickState = KickState.IDLE
    private val phaseTimer = ElapsedTime()

    private val LAUNCHER_POWER = 1.0
    private val intake_power = 0.65

    // ================= AprilTag Rotate (BLUE) =================
    private val TARGET_TAG_ID = 20
    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    // smoother controller
    private val TURN_KP = 0.012
    private val YAW_TOL_DEG = 3.0
    private val YAW_UNLOCK_DEG = 4.0

    private val MAX_TURN = 0.22

    // helps overcome static friction so it doesn't "stutter"
    private val MIN_TURN = 0.06

    // controller drift guard
    private val STICK_DEADBAND = 0.06

    // stability gate
    private val REQUIRED_SEEN_FRAMES = 4

    // once centered, hold still for a bit (prevents chatter)
    private val HOLD_MS = 250.0

    private fun initAprilTag() {
        aprilTag = AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .build()

        val cam = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        visionPortal = VisionPortal.Builder()
            .setCamera(cam)
            .addProcessor(aprilTag)
            .build()
    }

    private fun getTargetDetection(): AprilTagDetection? {
        for (d in aprilTag.detections) {
            if (d.id == TARGET_TAG_ID) return d
        }
        return null
    }
    // ==========================================================

    override fun runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
        backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
        frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
        backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")

        intakeRuns = hardwareMap.get(DcMotor::class.java, "intake_motor")
        launchMoto = hardwareMap.get(DcMotor::class.java, "launcher")
        pusher = hardwareMap.get(Servo::class.java, "servo_motor")
        hoodServoLeft = hardwareMap.get(Servo::class.java, "launcherHood_left")
        hoodServoRight = hardwareMap.get(Servo::class.java, "launcherHood_right")

        hoodServoLeft!!.direction = Servo.Direction.FORWARD
        hoodServoRight!!.direction = Servo.Direction.REVERSE

        hoodServoLeft!!.position = 0.12
        hoodServoRight!!.position = 0.12

        pusher!!.position = REST_POS

        frontLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive!!.direction = DcMotorSimple.Direction.FORWARD

        launchMoto!!.direction = DcMotorSimple.Direction.REVERSE
        intakeRuns!!.direction = DcMotorSimple.Direction.FORWARD

        initAprilTag()

        setManualExposure(5, 100)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        var prevY = gamepad2.y

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

        var alignEnabled = false
        var lockedOn = false

        // filtered yaw to reduce jitter
        var yawFiltered = 0.0

        // stable detection gate
        var seenCount = 0

        // lock hold timer
        val lockTimer = ElapsedTime()

        while (opModeIsActive()) {

            telemetry.addLine("frontLeft " + frontLeftDrive!!.currentPosition)
            telemetry.addLine("frontRight " + frontRightDrive!!.currentPosition)
            telemetry.addLine("backLeft " + backLeftDrive!!.currentPosition)
            telemetry.addLine("backRight " + backRightDrive!!.currentPosition)

            var axial = -gamepad1.left_stick_y.toDouble()
            var lateral = gamepad1.left_stick_x.toDouble()
            var yaw = gamepad1.right_stick_x.toDouble()

            if (abs(axial) < STICK_DEADBAND) axial = 0.0
            if (abs(lateral) < STICK_DEADBAND) lateral = 0.0
            if (abs(yaw) < STICK_DEADBAND) yaw = 0.0

            // Toggle align on Y press (gamepad2)
            val yNow = gamepad2.y
            if (yNow && !prevY) {
                alignEnabled = !alignEnabled
                lockedOn = false
                yawFiltered = 0.0
                seenCount = 0
                lockTimer.reset()
            }
            prevY = yNow

            val target = getTargetDetection()

            if (alignEnabled && target != null && target.ftcPose != null) {
                seenCount++
            } else {
                seenCount = 0
                lockedOn = false
            }

            if (alignEnabled && target != null && target.ftcPose != null && seenCount >= REQUIRED_SEEN_FRAMES) {
                val yawErrDegRaw = target.ftcPose.yaw

                // stronger smoothing
                yawFiltered = 0.85 * yawFiltered + 0.15 * yawErrDegRaw
                val yawErrDeg = yawFiltered

                val withinYaw = abs(yawErrDeg) <= YAW_TOL_DEG

                // lock when centered
                if (!lockedOn && withinYaw) {
                    lockedOn = true
                    lockTimer.reset()
                }

                // stay locked for HOLD_MS even if tiny noise appears
                val holding = lockedOn && lockTimer.milliseconds() < HOLD_MS

                // unlock only if we are clearly off for a bit
                if (lockedOn && !holding && abs(yawErrDeg) >= YAW_UNLOCK_DEG) {
                    lockedOn = false
                }

                yaw = if (withinYaw || holding) {
                    0.0
                } else {
                    // P + feedforward min turn so motors actually move smoothly
                    var cmd = -yawErrDeg * TURN_KP
                    if (abs(cmd) < MIN_TURN) cmd = MIN_TURN * sign(cmd)
                    cmd.coerceIn(-MAX_TURN, MAX_TURN)
                }
            }

            val bNow = gamepad2.b
            if (bNow && !prevB) launcherEnabled = !launcherEnabled
            prevB = bNow

            val aNow = gamepad2.a
            if (aNow && !prevA) intakeEnabled = !intakeEnabled
            prevA = aNow

            var frontLeftPower = axial + lateral + yaw
            var frontRightPower = axial - lateral - yaw
            var backLeftPower = axial - lateral + yaw
            var backRightPower = axial + lateral - yaw

            var maxPow = max(abs(frontLeftPower), abs(frontRightPower))
            maxPow = max(maxPow, abs(backLeftPower))
            maxPow = max(maxPow, abs(backRightPower))

            if (maxPow > 1.0) {
                frontLeftPower /= maxPow
                frontRightPower /= maxPow
                backLeftPower /= maxPow
                backRightPower /= maxPow
            }

            frontLeftDrive!!.power = frontLeftPower
            frontRightDrive!!.power = frontRightPower
            backLeftDrive!!.power = backLeftPower
            backRightDrive!!.power = backRightPower

            launchMoto!!.power = if (launcherEnabled) LAUNCHER_POWER else 0.0
            intakeRuns!!.power = if (intakeEnabled) intake_power else 0.0

            val xNow = gamepad2.x
            if (xNow && !prevX && kickState == KickState.IDLE) {
                pusher!!.position = KICK_POS
                phaseTimer.reset()
                kickState = KickState.EXTENDING
            }
            prevX = xNow

            when (kickState) {
                KickState.EXTENDING -> {
                    if (phaseTimer.milliseconds() >= KICK_TIME_MS.toDouble()) {
                        pusher!!.position = REST_POS
                        kickState = KickState.IDLE
                    }
                }
                else -> {}
            }

            manageHood();

            telemetry.addData("Info", "Run Time: $runtime")
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower)
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower)
            telemetry.addData("Launcher", "%4.2f (%s)", LAUNCHER_POWER, if (launcherEnabled) "ON" else "OFF")
            telemetry.addData("Intake", "%4.2f (%s)", intake_power, if (intakeEnabled) "ON" else "OFF")
            telemetry.addData("Pusher", "state=%s pos=%.2f", kickState, pusher!!.position)

            telemetry.addData("Align", "Y Toggle: %s", if (alignEnabled) "ON" else "OFF")
            telemetry.addData("Lock", "%s", if (lockedOn) "LOCKED" else "SEEK")
            telemetry.addData("SeenFrames", "%d/%d", seenCount, REQUIRED_SEEN_FRAMES)

            if (target != null && target.ftcPose != null) {
                telemetry.addData("Tag", "BLUE id=%d seen", TARGET_TAG_ID)
                telemetry.addData("Pose", "yawRaw=%.1f yawF=%.1f y=%.1f z=%.1f",
                    target.ftcPose.yaw, yawFiltered, target.ftcPose.y, target.ftcPose.z)
            } else {
                telemetry.addData("Tag", "BLUE id=%d not seen", TARGET_TAG_ID)
            }

            telemetry.update()
        }

        if (::visionPortal.isInitialized) visionPortal.close()
    }

    private fun manageHood(){
        var dPadUp = gamepad2.dpad_up
        var dPadDown = gamepad2.dpad_down

        if (dPadUp) {
            if (hoodServoLeft!!.position > 0.12 || hoodServoRight!!.position > 0.12) return
            hoodServoLeft!!.position += 0.005
            hoodServoRight!!.position += 0.005
        }

//        1 = left
//        2 = right

        else if (dPadDown) {
            if (hoodServoLeft!!.position <= .003 || hoodServoRight!!.position <= .003) return
            hoodServoLeft!!.position -= 0.005
            hoodServoRight!!.position -= 0.005
        }
    }

    private fun setManualExposure(exposureMS: Int, gain: Int) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting")
            telemetry.update()
            while (!isStopRequested && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20)
            }
            telemetry.addData("Camera", "Ready")
            telemetry.update()
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            val exposureControl =
                visionPortal!!.getCameraControl<ExposureControl>(ExposureControl::class.java)
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual)
                sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)
            val gainControl = visionPortal!!.getCameraControl<GainControl>(GainControl::class.java)
            gainControl.setGain(gain)
            sleep(20)
        }
    }
}