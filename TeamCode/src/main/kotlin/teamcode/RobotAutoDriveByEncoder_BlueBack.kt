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
package teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.PI

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
@Autonomous(name = "Auto Drive By Encoder Blue Back", group = "Robot")
class RobotAutoDriveByEncoder_BlueBack : LinearOpMode() {
    /* Declare OpMode members. */
    val DESIRED_DISTANCE: Double =
        36.0 //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    val SPEED_GAIN: Double =
        0.02 //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    val STRAFE_GAIN: Double =
        0.015 //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    val TURN_GAIN: Double =
        0.01 //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    val MAX_AUTO_SPEED: Double =
        0.5 //  Clip the approach speed to this max value (adjust for your robot)
    val MAX_AUTO_STRAFE: Double =
        0.5 //  Clip the strafing speed to this max value (adjust for your robot)
    val MAX_AUTO_TURN: Double =
        0.3 //  Clip the turn speed to this max value (adjust for your robot)
    private lateinit var frontLeftDrive: DcMotor
    private lateinit var backLeftDrive: DcMotor
    private lateinit var frontRightDrive: DcMotor
    private lateinit var backRightDrive: DcMotor
    private lateinit var launcherMotor: DcMotor
    private lateinit var intakeMotor: DcMotor
    private lateinit var servo: Servo
    private var visionPortal: VisionPortal? = null // Used to manage the video source.
    private var aprilTag: AprilTagProcessor? =
        null // Used for managing the AprilTag detection process.
    private var desiredTag: AprilTagDetection? =
        null // Used to hold the data for a detected AprilTag


    override fun runOpMode() {
        var targetFound = false // Set to true when an AprilTag target is detected
        var drive = 0.0 // Desired forward power/speed (-1 to +1)
        var strafe = 0.0 // Desired strafe power/speed (-1 to +1)
        var turn = 0.0 // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag()

        runBlocking {
            // Initialize the drive system variables.
            frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
            backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
            frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
            backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")
            launcherMotor = hardwareMap.get(DcMotor::class.java, "launcher")
            intakeMotor = hardwareMap.get(DcMotor::class.java, "intake_motor")
            servo = hardwareMap.get(Servo::class.java, "servo_motor")

            setManualExposure(1, 100) // Use low exposure time to reduce motion blur

            waitForStart()

            val chassis = Chassis(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                this@RobotAutoDriveByEncoder_BlueBack
            )
            chassis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            chassis.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            servo.direction = Servo.Direction.FORWARD
            intakeMotor.power = -0.6

            chassis.encoderHoris(-2.0)
            chassis.encoderVert(36.0)
            chassis.encoderDiagonal(-42.0, true)
            chassis.encoderRotationRadians((3 * PI) / 4, 0.4)

            servo.position = 0.5
            launcherMotor.power = 1.0
            sleep(3000)
            servo.position = 1.0
            sleep(1000)
            launcherMotor.power = 0.0


            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
            telemetry.addData(">", "Touch START to start OpMode")
            telemetry.update()
            waitForStart()

            while (opModeIsActive()) {
                targetFound = false
                desiredTag = null

                // Step through the list of detected tags and look for a matching tag
                val currentDetections: MutableList<AprilTagDetection> = aprilTag!!.getDetections()
                for (detection in currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true
                            desiredTag = detection
                            break // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id)
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id)
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n")
                    telemetry.addData(
                        "Found",
                        "ID %d (%s)",
                        desiredTag!!.id,
                        desiredTag!!.metadata.name
                    )
                    telemetry.addData("Range", "%5.1f inches", desiredTag!!.ftcPose.range)
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag!!.ftcPose.bearing)
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag!!.ftcPose.yaw)
                } else {
                    telemetry.addData("\n>", "Drive using joysticks to find valid target\n")
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (gamepad1.left_bumper && targetFound) {
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.

                    val rangeError = (desiredTag!!.ftcPose.range - DESIRED_DISTANCE)
                    val headingError = desiredTag!!.ftcPose.bearing
                    val yawError = desiredTag!!.ftcPose.yaw

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)

                    telemetry.addData(
                        "Auto",
                        "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                        drive,
                        strafe,
                        turn
                    )
                } else {
                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.

                    drive = -gamepad1.left_stick_y / 2.0 // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x / 2.0 // Reduce strafe rate to 50%.
                    turn = -gamepad1.right_stick_x / 3.0 // Reduce turn rate to 33%.
                    telemetry.addData(
                        "Manual",
                        "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                        drive,
                        strafe,
                        turn
                    )
                }
                telemetry.update()

                // Apply desired axes motions to the drivetrain.
                chassis.figureOutPower(drive, strafe, turn)
                sleep(10)
            }
        }
    }

    private fun initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = AprilTagProcessor.Builder().build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag!!.setDecimation(2f)

        // Create the vision portal by using a builder.
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()

    }

    private fun setManualExposure(exposureMS: Int, gain: Int) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal!!.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting")
            telemetry.update()
            while (!isStopRequested() && (visionPortal!!.getCameraState() != VisionPortal.CameraState.STREAMING)) {
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


    companion object {
        private val DESIRED_TAG_ID =
            20 // Choose the tag you want to approach or set to -1 for ANY tag.
    }
}