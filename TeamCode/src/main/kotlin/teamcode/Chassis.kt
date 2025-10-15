package teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max

class Chassis(private val frontLeft: DcMotor, private val frontRight: DcMotor, private val backLeft: DcMotor, private val backRight: DcMotor, val instance: LinearOpMode) {
    val runtime = ElapsedTime()

    init {
        frontLeft!!.direction = DcMotorSimple.Direction.REVERSE
        backLeft!!.direction = DcMotorSimple.Direction.REVERSE
        frontRight!!.direction = DcMotorSimple.Direction.FORWARD
        backRight!!.direction = DcMotorSimple.Direction.FORWARD
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