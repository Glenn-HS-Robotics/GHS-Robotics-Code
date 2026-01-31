package teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import teamcode.Chassis

@Autonomous(name = "ParkingToRed", group = "Linear OpMode")
class ParkingToRed : LinearOpMode() {

    private lateinit var frontLeft: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backRight: DcMotor

    private val TICKS_PER_REV = 312.0
    private val WHEEL_DIAMETER = 4.0
    private val GEAR_RATIO = 1.0

    private val MAX_SPEED = 0.6
    private val ACCEL_STEP = 0.05

    override fun runOpMode() {

        frontLeft = hardwareMap.get(DcMotor::class.java, "front_left")
        backLeft = hardwareMap.get(DcMotor::class.java, "back_left")
        frontRight = hardwareMap.get(DcMotor::class.java, "front_right")
        backRight = hardwareMap.get(DcMotor::class.java, "back_right")

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        frontRight.direction = DcMotorSimple.Direction.FORWARD
        backRight.direction = DcMotorSimple.Direction.FORWARD

        val chassis = Chassis(frontLeft, frontRight, backLeft, backRight, this)

        resetEncoders()

        telemetry.addLine("Blue Auto Initialized")
        telemetry.update()

        waitForStart()

        chassis.encoderVert(28.0, .5)

        chassis.encoderHoris(-36.0, .5)

        telemetry.addLine("Blue Auto Complete")
        telemetry.update()
        sleep(500)
    }

    private fun inchesToTicks(inches: Double): Int {
        val rotations = inches / (Math.PI * WHEEL_DIAMETER) * GEAR_RATIO
        return (rotations * TICKS_PER_REV).toInt()
    }

    private fun resetEncoders() {
        frontLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private fun encoderMove(forward: Double, strafe: Double) {

        // Correct mecanum math
        val flTarget = frontLeft.currentPosition + inchesToTicks(forward + strafe)
        val frTarget = frontRight.currentPosition + inchesToTicks(forward - strafe)
        val blTarget = backLeft.currentPosition + inchesToTicks(forward + strafe)
        val brTarget = backRight.currentPosition + inchesToTicks(forward - strafe)

        frontLeft.targetPosition = flTarget
        frontRight.targetPosition = frTarget
        backLeft.targetPosition = blTarget
        backRight.targetPosition = brTarget

        frontLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        frontRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        backLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        backRight.mode = DcMotor.RunMode.RUN_TO_POSITION

        var power = 0.0

        while (opModeIsActive() &&
            (frontLeft.isBusy || frontRight.isBusy || backLeft.isBusy || backRight.isBusy)) {

            if (power < MAX_SPEED) power += ACCEL_STEP
            if (power > MAX_SPEED) power = MAX_SPEED

            frontLeft.power = power
            frontRight.power = power
            backLeft.power = power
            backRight.power = power
        }

        stopAll()
    }

    private fun stopAll() {
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0

        frontLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}