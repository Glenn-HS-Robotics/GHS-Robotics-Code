package teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max

const val COUNTS_PER_MOTOR_REV = 2150.4

const val DRIVE_GEAR_REDUCTION = 1.0

const val WHEEL_DIAMETER_INCHES = 4.09

const val COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
        (WHEEL_DIAMETER_INCHES * 3.1415)

class Chassis(private val frontLeft: DcMotor, private val frontRight: DcMotor, private val backLeft: DcMotor, private val backRight: DcMotor, val instance: LinearOpMode) {
    val runtime = ElapsedTime()

    init {
        frontLeft!!.direction = DcMotorSimple.Direction.REVERSE
        backLeft!!.direction = DcMotorSimple.Direction.REVERSE
        frontRight!!.direction = DcMotorSimple.Direction.FORWARD
        backRight!!.direction = DcMotorSimple.Direction.FORWARD
        setMode(RunMode.RUN_USING_ENCODER)
    }

    fun encoderVert(inches: Double){

        // Determine new target position, and pass to motor controller
        val newTarget = frontLeft.currentPosition + (inches * COUNTS_PER_INCH).toInt()
        frontLeft.targetPosition = newTarget
        frontRight.targetPosition = newTarget
        backLeft.targetPosition = newTarget
        backRight.targetPosition = newTarget
        setMode(RunMode.RUN_TO_POSITION)

        power(0.6)

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (instance.opModeIsActive() && frontLeft.isBusy && frontRight.isBusy) {

            // Display it for the driver.
            instance.telemetry.addData("Running to", " %7d", newTarget )
            instance.telemetry.addData(
                "Currently at", " at front: %7d  back: %7d",
                frontLeft.currentPosition, backRight.currentPosition
            )
            instance.telemetry.update()
        }
        stop()
        setMode(RunMode.RUN_USING_ENCODER)
        instance.telemetry.addData("Path", "Complete")
        instance.telemetry.update()
        instance.sleep(100)

    }

    fun encoderHoris(inches: Double){

        // Determine new target position, and pass to motor controller
        val newTarget = frontLeft.currentPosition + (inches * COUNTS_PER_INCH).toInt()
        frontLeft.targetPosition = newTarget
        frontRight.targetPosition = -newTarget
        backLeft.targetPosition = -newTarget
        backRight.targetPosition = newTarget
        setMode(RunMode.RUN_TO_POSITION)

        power(0.6)

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (instance.opModeIsActive() && frontLeft.isBusy && frontRight.isBusy) {

            // Display it for the driver.
            instance.telemetry.addData("Running to", " %7d :%7d", newTarget, newTarget)
            instance.telemetry.addData(
                "Currently at", " at front: %7d  back: %7d",
                frontLeft.currentPosition, backRight.currentPosition
            )
            instance.telemetry.update()
        }
        stop()
        setMode(RunMode.RUN_USING_ENCODER)
        instance.telemetry.addData("Path", "Complete")
        instance.telemetry.update()
        instance.sleep(100)

    }

    fun vert(power: Double){
        figureOutPower(power, 0.0, 0.0)
    }

    fun horis(power: Double){
        figureOutPower(0.0, power, 0.0)
    }

    fun rotation(yaw: Double){
        figureOutPower(0.0, 0.0, yaw)
    }

    fun stop(){
        figureOutPower(0.0,0.0,0.0)
    }

    fun setMode(runMode: RunMode){
        frontRight.mode = runMode
        frontLeft.mode = runMode
        backRight.mode = runMode
        backLeft.mode = runMode
    }

    fun power(power: Double){
        frontLeft.power = power
        frontRight.power = power
        backLeft.power = power
        backRight.power = power
    }

    fun wait(time: Double){
        runtime.reset()
        while (instance.opModeIsActive() && runtime.seconds() < time) {
            instance.telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds())
            instance.telemetry.update()
        }
    }

    private fun figureOutPower(axial: Double, lateral: Double, yaw: Double){
        var frontLeftPower = axial + lateral + yaw
        var frontRightPower = axial - lateral - yaw
        var backLeftPower = axial - lateral + yaw
        var backRightPower = axial + lateral - yaw
        var max = 0.0

        max = max(abs(frontLeftPower), abs(frontRightPower))
        max = max(max, abs(backLeftPower))
        max = max(max, abs(backRightPower))

        if (max > 1.0) {
            frontLeftPower /= max
            frontRightPower /= max
            backLeftPower /= max
            backRightPower /= max
        }

        frontLeft.power = frontLeftPower
        frontRight.power = frontRightPower
        backLeft.power = backLeftPower
        backRight.power = backRightPower
    }


}