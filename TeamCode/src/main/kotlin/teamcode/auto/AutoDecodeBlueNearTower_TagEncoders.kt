package teamcode.auto

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
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
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sin

@Autonomous(name = "AUTO: DECODE BLUE Near Tower", group = "Linear OpMode")
class AutoDecodeBlueNearTower_TagEncoders : LinearOpMode() {

    // ---------------- Hardware ----------------
    private var frontLeftDrive: DcMotorEx? = null
    private var backLeftDrive: DcMotorEx? = null
    private var frontRightDrive: DcMotorEx? = null
    private var backRightDrive: DcMotorEx? = null

    private var launchMoto: DcMotorEx? = null
    private var intakeRuns: DcMotorEx? = null

    private var hoodServoLeft: Servo? = null
    private var hoodServoRight: Servo? = null
    private var pusher: Servo? = null

    private lateinit var imu: BNO055IMU

    // ---------------- Shooter / Intake ----------------
    private val REST_POS = 1.0
    private val KICK_POS = 0.3
    private val KICK_TIME_MS = 450L
    private val intake_power = 0.65

    // ---------------- AprilTag ----------------
    private val TARGET_TAG_ID = 20 // BLUE tag id in your TeleOp
    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    private val TURN_KP_TAG = 0.012
    private val YAW_TOL_DEG = 3.0
    private val YAW_UNLOCK_DEG = 4.0
    private val MAX_TURN_TAG = 0.22
    private val MIN_TURN_TAG = 0.06
    private val REQUIRED_SEEN_FRAMES = 4
    private val HOLD_MS = 250.0

    // Range used for hood/power curves (same as your TeleOp)
    private var distancePlanar = 50.0

    // ---------------- Pose ----------------
    /*
      Coordinate convention (IMPORTANT):
      This assumes a common field frame:
        +X = forward, inches
        +Y = left, inches
        headingDeg = 0 faces +X, CCW positive

      If your coordinate/heading system is different, the driveToPose transform may need sign changes.
    */
    private var poseX = 20.0
    private var poseY = 120.0
    private var poseHeadingDeg = 140.0

    // Offset to force IMU heading to match your requested start heading
    private var headingOffsetDeg = 0.0

    // Encoder deltas
    private var lastFL = 0
    private var lastFR = 0
    private var lastBL = 0
    private var lastBR = 0

    // ---------- TUNING CONSTANTS (VERIFY) ----------
    private val WHEEL_DIAMETER_IN = 3.7795
    private val GEAR_RATIO = 1.0
    private val TICKS_PER_MOTOR_REV = 537.7
    private val COUNTS_PER_IN =
        (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN)

    // Slow + safe motion
    private val MAX_TRANSLATE_POWER = 0.30
    private val MAX_ROTATE_POWER = 0.22
    private val KP_POS = 0.045
    private val KP_HEADING = 0.010
    private val MIN_TURN = 0.05
    private val POS_TOL_IN = 1.5
    private val HEADING_TOL_DEG = 3.0

    override fun runOpMode() {
        initHardware()
        initAprilTag()
        setManualExposure(5, 100)

        pusher!!.position = REST_POS
        hoodServoLeft!!.position = 0.12
        hoodServoRight!!.position = 0.12

        resetDriveEncoders()
        seedPoseFromStart(20.0, 120.0, 140.0)

        telemetry.addLine("Initialized Auto")
        telemetry.addData("Start Pose", "(%.1f, %.1f) h=%.1f", poseX, poseY, poseHeadingDeg)
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        // Force IMU heading reference to match your starting heading
        syncHeadingOffsetToStartHeading(140.0)

        // Shoot 2 preloaded
        aimAndShootAtTag(shots = 2, spinupMs = 650)

        // 3 cycles
        val pickupPoints = listOf(
            Triple(24.0, 84.0, 180.0),
            Triple(24.0, 60.0, 180.0),
            Triple(24.0, 36.0, 180.0)
        )

        for ((px, py, ph) in pickupPoints) {
            driveToPose(px, py, ph, timeoutS = 6.0)
            collectBallsAtPoint(collectMs = 1200)

            driveToPose(48.0, 96.0, 140.0, timeoutS = 7.0)
            aimAndShootAtTag(shots = 2, spinupMs = 550)
        }

        stopAll()
        if (::visionPortal.isInitialized) visionPortal.close()
    }

    // ---------------- Init ----------------
    private fun initHardware() {
        frontLeftDrive = hardwareMap.get(DcMotorEx::class.java, "front_left")
        backLeftDrive = hardwareMap.get(DcMotorEx::class.java, "back_left")
        frontRightDrive = hardwareMap.get(DcMotorEx::class.java, "front_right")
        backRightDrive = hardwareMap.get(DcMotorEx::class.java, "back_right")

        intakeRuns = hardwareMap.get(DcMotorEx::class.java, "intake_motor")
        launchMoto = hardwareMap.get(DcMotorEx::class.java, "launcher")

        pusher = hardwareMap.get(Servo::class.java, "servo_motor")
        hoodServoLeft = hardwareMap.get(Servo::class.java, "launcherHood_left")
        hoodServoRight = hardwareMap.get(Servo::class.java, "launcherHood_right")

        hoodServoLeft!!.direction = Servo.Direction.FORWARD
        hoodServoRight!!.direction = Servo.Direction.REVERSE

        frontLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive!!.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive!!.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive!!.direction = DcMotorSimple.Direction.FORWARD

        launchMoto!!.direction = DcMotorSimple.Direction.REVERSE
        intakeRuns!!.direction = DcMotorSimple.Direction.FORWARD

        listOf(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive).forEach {
            it!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        // ---- IMU (FIXED): must be initialized before use ----
        imu = hardwareMap.get(BNO055IMU::class.java, "imu") // you renamed to "imu"
        val params = BNO055IMU.Parameters().apply {
            angleUnit = BNO055IMU.AngleUnit.DEGREES
        }
        imu.initialize(params)
        // ------------------------------------------------------
    }

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

    // ---------------- Pose helpers ----------------
    private fun seedPoseFromStart(x: Double, y: Double, headingDeg: Double) {
        poseX = x
        poseY = y
        poseHeadingDeg = headingDeg
    }

    private fun syncHeadingOffsetToStartHeading(desiredHeadingDeg: Double) {
        val imuRaw = imu.angularOrientation.firstAngle.toDouble()
        headingOffsetDeg = angleWrapDeg(desiredHeadingDeg - imuRaw)
        poseHeadingDeg = desiredHeadingDeg
    }

    private fun getHeadingDeg(): Double {
        val imuRaw = imu.angularOrientation.firstAngle.toDouble()
        return angleWrapDeg(imuRaw + headingOffsetDeg)
    }

    private fun resetDriveEncoders() {
        listOf(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive).forEach {
            it!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        lastFL = 0
        lastFR = 0
        lastBL = 0
        lastBR = 0
    }

    private fun updatePoseFromEncoders() {
        val fl = frontLeftDrive!!.currentPosition
        val fr = frontRightDrive!!.currentPosition
        val bl = backLeftDrive!!.currentPosition
        val br = backRightDrive!!.currentPosition

        val dfl = (fl - lastFL).toDouble()
        val dfr = (fr - lastFR).toDouble()
        val dbl = (bl - lastBL).toDouble()
        val dbr = (br - lastBR).toDouble()

        lastFL = fl
        lastFR = fr
        lastBL = bl
        lastBR = br

        // Robot-frame deltas (inches)
        val forwardIn = (dfl + dfr + dbl + dbr) / 4.0 / COUNTS_PER_IN
        val leftIn = (-dfl + dfr + dbl - dbr) / 4.0 / COUNTS_PER_IN

        // Field-frame integration using IMU heading
        val h = Math.toRadians(getHeadingDeg())
        val cosH = cos(h)
        val sinH = sin(h)

        poseX += forwardIn * cosH - leftIn * sinH
        poseY += forwardIn * sinH + leftIn * cosH
        poseHeadingDeg = getHeadingDeg()
    }

    // ---------------- Drive control ----------------
    private fun driveToPose(
        targetX: Double,
        targetY: Double,
        targetHeadingDeg: Double,
        timeoutS: Double
    ) {
        val timer = ElapsedTime()
        timer.reset()

        while (opModeIsActive() && timer.seconds() < timeoutS) {
            updatePoseFromEncoders()

            val dx = targetX - poseX
            val dy = targetY - poseY
            val dist = hypot(dx, dy)

            val heading = getHeadingDeg()
            val h = Math.toRadians(heading)
            val cosH = cos(h)
            val sinH = sin(h)

            // field error -> robot frame
            val forwardErr = dx * cosH + dy * sinH
            val leftErr = -dx * sinH + dy * cosH

            // your TeleOp uses lateralRight positive, so invert leftErr
            var axial = (forwardErr * KP_POS).coerceIn(-MAX_TRANSLATE_POWER, MAX_TRANSLATE_POWER)
            var lateralRight = (-leftErr * KP_POS).coerceIn(-MAX_TRANSLATE_POWER, MAX_TRANSLATE_POWER)

            val headingErr = angleWrapDeg(targetHeadingDeg - heading)
            var yaw = (headingErr * KP_HEADING).coerceIn(-MAX_ROTATE_POWER, MAX_ROTATE_POWER)
            if (abs(yaw) < MIN_TURN && abs(headingErr) > HEADING_TOL_DEG) yaw = MIN_TURN * sign(yaw)

            // slow down near target
            val slowFactor = (dist / 18.0).coerceIn(0.25, 1.0)
            axial *= slowFactor
            lateralRight *= slowFactor

            setDrivePowers(axial, lateralRight, yaw)

            telemetry.addData("DriveTo", "Target (%.1f, %.1f) h=%.1f", targetX, targetY, targetHeadingDeg)
            telemetry.addData("Pose", "(%.1f, %.1f) h=%.1f", poseX, poseY, heading)
            telemetry.addData("Err", "dist=%.1f in, headErr=%.1f", dist, headingErr)
            telemetry.update()

            val donePos = dist <= POS_TOL_IN
            val doneHeading = abs(headingErr) <= HEADING_TOL_DEG
            if (donePos && doneHeading) break

            sleep(20)
        }

        setDrivePowers(0.0, 0.0, 0.0)
        sleep(120)
    }

    private fun setDrivePowers(axial: Double, lateralRight: Double, yaw: Double) {
        var fl = axial + lateralRight + yaw
        var fr = axial - lateralRight - yaw
        var bl = axial - lateralRight + yaw
        var br = axial + lateralRight - yaw

        val maxMag = max(1.0, max(max(abs(fl), abs(fr)), max(abs(bl), abs(br))))
        fl /= maxMag
        fr /= maxMag
        bl /= maxMag
        br /= maxMag

        frontLeftDrive!!.power = fl
        frontRightDrive!!.power = fr
        backLeftDrive!!.power = bl
        backRightDrive!!.power = br
    }

    // ---------------- Collect ----------------
    private fun collectBallsAtPoint(collectMs: Long) {
        intakeRuns!!.power = intake_power

        // gentle creep forward to help pickup
        setDrivePowers(0.10, 0.0, 0.0)
        sleep((collectMs * 0.6).toLong())

        setDrivePowers(0.0, 0.0, 0.0)
        sleep((collectMs * 0.4).toLong())

        intakeRuns!!.power = 0.0
    }

    // ---------------- Aim + Shoot ----------------
    private fun getTargetDetection(): AprilTagDetection? {
        for (d in aprilTag.detections) {
            if (d.id == TARGET_TAG_ID) return d
        }
        return null
    }

    private fun aimAndShootAtTag(shots: Int, spinupMs: Long) {
        val lockTimer = ElapsedTime()
        val overallTimer = ElapsedTime()

        var yawFiltered = 0.0
        var seenCount = 0
        var lockedOn = false

        lockTimer.reset()
        overallTimer.reset()

        // Start with shooter off, then enable once tag is seen (so range-based power is valid)
        launchMoto!!.velocity = 0.0
        sleep(80)

        while (opModeIsActive() && overallTimer.seconds() < 4.0) {
            val target = getTargetDetection()
            if (target != null && target.ftcPose != null) {
                distancePlanar = target.ftcPose.range
                seenCount++
            } else {
                seenCount = 0
                lockedOn = false
                launchMoto!!.velocity = 0.0
                setDrivePowers(0.0, 0.0, 0.0)
                telemetry.addLine("Tag not seen - holding")
                telemetry.update()
                sleep(40)
                continue
            }

            // Update hood and shooter based on range
            val hoodHoodedness = getHoodHoodedness()
            val angle = (hoodHoodedness - 0.003) * 0.12
            hoodServoLeft!!.position = angle
            hoodServoRight!!.position = angle

            val launcherVel = getLauncherPower() * 2450.0
            launchMoto!!.velocity = launcherVel

            if (seenCount < REQUIRED_SEEN_FRAMES) {
                setDrivePowers(0.0, 0.0, 0.0)
                sleep(30)
                continue
            }

            val yawErrDegRaw = target.ftcPose.yaw
            yawFiltered = 0.85 * yawFiltered + 0.15 * yawErrDegRaw
            val yawErrDeg = yawFiltered

            val withinYaw = abs(yawErrDeg) <= YAW_TOL_DEG

            if (!lockedOn && withinYaw) {
                lockedOn = true
                lockTimer.reset()
            }

            val holding = lockedOn && lockTimer.milliseconds() < HOLD_MS
            if (lockedOn && !holding && abs(yawErrDeg) >= YAW_UNLOCK_DEG) {
                lockedOn = false
            }

            val yawCmd = if (withinYaw || holding) {
                0.0
            } else {
                var cmd = -yawErrDeg * TURN_KP_TAG
                if (abs(cmd) < MIN_TURN_TAG) cmd = MIN_TURN_TAG * sign(cmd)
                cmd.coerceIn(-MAX_TURN_TAG, MAX_TURN_TAG)
            }

            setDrivePowers(0.0, 0.0, yawCmd)

            telemetry.addData("Aim", "yawRaw=%.1f yawF=%.1f", yawErrDegRaw, yawFiltered)
            telemetry.addData("Range", "%.1f", distancePlanar)
            telemetry.addData("Locked", lockedOn)
            telemetry.update()

            if (lockedOn && lockTimer.milliseconds() >= HOLD_MS) break
            sleep(20)
        }

        setDrivePowers(0.0, 0.0, 0.0)
        sleep(120)

        // Let flywheel stabilize
        sleep(spinupMs)

        repeat(shots) {
            kickOnce()
            sleep(260)
        }

        launchMoto!!.velocity = 0.0
        sleep(100)
    }

    private fun kickOnce() {
        pusher!!.position = KICK_POS
        sleep(KICK_TIME_MS)
        pusher!!.position = REST_POS
    }

    // ---------------- Camera exposure ----------------
    private fun setManualExposure(exposureMS: Int, gain: Int) {
        if (!::visionPortal.isInitialized) return

        if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested && visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
                sleep(20)
            }
        }

        if (!isStopRequested) {
            val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
            if (exposureControl.mode != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual)
                sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)

            val gainControl = visionPortal.getCameraControl(GainControl::class.java)
            gainControl.gain = gain
            sleep(20)
        }
    }

    // ---------------- Your existing curves ----------------
    private fun getLauncherPower(): Double {
        val normalized = (distancePlanar - 30.0) / 70.0
        val power = when {
            normalized < 0.15 -> 0.55 + (normalized * 0.15)
            normalized < 0.25 -> 0.43 + (normalized * 0.15)
            normalized < 1.0 -> 0.36 + (normalized * 0.20)
            else -> 0.44 + (normalized * 0.20)
        }
        telemetry.addData("Tag distance", "distance=%.1f", distancePlanar)
        telemetry.addData("Tag power", "power=%.3f", power)
        return power
    }

    private fun getHoodHoodedness(): Double {
        val normalized = (distancePlanar - 30.0) / 70.0
        if (normalized > 0.5) return 1.0
        val angledness = (normalized / 0.5)
        telemetry.addData("Angledness amount", "hood=%.3f", angledness)
        return angledness
    }

    // ---------------- Utilities ----------------
    private fun angleWrapDeg(deg: Double): Double {
        var a = deg
        while (a <= -180) a += 360.0
        while (a > 180) a -= 360.0
        return a
    }

    private fun stopAll() {
        setDrivePowers(0.0, 0.0, 0.0)
        intakeRuns!!.power = 0.0
        launchMoto!!.velocity = 0.0
        pusher!!.position = REST_POS
    }
}