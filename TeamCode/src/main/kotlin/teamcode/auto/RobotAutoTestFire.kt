package teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import teamcode.Chassis

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Auto Test Fire", group = "Robot")
class RobotAutoTestFire : LinearOpMode() {
    /* Declare OpMode members. */
    private lateinit var frontLeftDrive: DcMotor
    private lateinit var backLeftDrive: DcMotor
    private lateinit var frontRightDrive: DcMotor
    private lateinit var backRightDrive: DcMotor
    private lateinit var launcherMotor: DcMotor
    private lateinit var intakeMotor: DcMotor
    private lateinit var servo: Servo

    var gamepadBEnabled = false;




    override fun runOpMode() {
        runBlocking {
            // Initialize the drive system variables.
            frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
            backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
            frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
            backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")
            launcherMotor = hardwareMap.get(DcMotor::class.java, "launcher")
            intakeMotor = hardwareMap.get(DcMotor::class.java, "intake_motor")
            servo = hardwareMap.get(Servo::class.java, "servo_motor")

            waitForStart()

            val chassis = Chassis(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                this@RobotAutoTestFire
            )
            chassis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            chassis.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            servo.direction = Servo.Direction.FORWARD
            //intakeMotor.power = -0.6

            launch {
                while (opModeIsActive()) {
                    telemetry.addData("Info", "running A loop")
                    if (gamepad1.a) {
                        telemetry.addData("Info", "pressed A")
                        telemetry.update()
                        servo.position = 0.5
                        delay(3000)
                        servo.position = 1.0
                    }
                    delay(10)
                }
            }
            launch {
                while (opModeIsActive()) {
                    telemetry.addData("Info", "running B loop")
                    if (gamepad1.b != gamepadBEnabled) {
                        gamepadBEnabled = gamepad1.b
                        launcherMotor.power = if (gamepad1.b) -1.0 else 0.0
                        telemetry.addData("Info", "pressing B ${gamepad1.b}")
                        telemetry.update()
                    }
                    delay(10)
                }
            }

            while (opModeIsActive()) {
                telemetry.update()
                delay(50)
            }


        }
    }

}