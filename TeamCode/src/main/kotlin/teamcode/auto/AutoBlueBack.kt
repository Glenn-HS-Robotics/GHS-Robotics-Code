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
package teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import teamcode.Chassis
import teamcode.util.PolynomialApproximation
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.pow

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
@Autonomous(name = "Final Blue Back", group = "Robot")
class AutoBlueBack : LinearOpMode() {
    /* Declare OpMode members. */
    private lateinit var frontLeftDrive: DcMotor
    private lateinit var backLeftDrive: DcMotor
    private lateinit var frontRightDrive: DcMotor
    private lateinit var backRightDrive: DcMotor
    private lateinit var launcherMotor: DcMotor
    private lateinit var intakeMotor: DcMotor
    private lateinit var hoodServoLeft: Servo
    private lateinit var hoodServoRight: Servo
    private lateinit var pusher: Servo
    private var distancePlanar = 100.0
    private val TARGET_TAG_ID = 20
    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    private val REST_POS = 1.0
    private val KICK_POS = 0.2

    private val powerApproximator =
        PolynomialApproximation(
            -66.92431823,
            10.78093689,
            -0.7497408548,
            0.02987215741,
            -0.0007562628741,
            0.00001273064853,
            -1.445116245 * 10.0.pow(-7),
            1.093739166 * 10.0.pow(-9),
            -5.288811859 * 10.0.pow(-12),
            1.477184641 * 10.0.pow(-14),
            -1.811606723 * 10.0.pow(-17)
        )

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

    override fun runOpMode() {
        runBlocking {
            // Initialize the drive system variables.
            frontLeftDrive = hardwareMap.get(DcMotor::class.java, "front_left")
            backLeftDrive = hardwareMap.get(DcMotor::class.java, "back_left")
            frontRightDrive = hardwareMap.get(DcMotor::class.java, "front_right")
            backRightDrive = hardwareMap.get(DcMotor::class.java, "back_right")
            launcherMotor = hardwareMap.get(DcMotor::class.java, "launcher")
            intakeMotor = hardwareMap.get(DcMotor::class.java, "intake_motor")
            pusher = hardwareMap.get(Servo::class.java, "servo_motor")
            hoodServoLeft = hardwareMap.get(Servo::class.java, "launcherHood_left")
            hoodServoRight = hardwareMap.get(Servo::class.java, "launcherHood_right")
            hoodServoLeft.direction = Servo.Direction.FORWARD
            hoodServoRight.direction = Servo.Direction.REVERSE
            hoodServoLeft.position = 0.13
            hoodServoRight.position = 0.13
            launcherMotor.direction = DcMotorSimple.Direction.REVERSE

            waitForStart()
            initAprilTag()
            setManualExposure(5, 100)

            val chassis = Chassis(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                this@AutoBlueBack
            )
            chassis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            chassis.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            pusher.direction = Servo.Direction.FORWARD
            //intakeMotor.power = 1.0

            chassis.encoderVert(134.0, 0.6)
            detectCode()
            chassis.stop()
            sleep(500)
            chassis.encoderRotationRadians(-PI / 1.7, 0.3)
            sleep(500)

            detectCode()
            detectCode()
            detectCode()
            setHoodAngle()
            launcherMotor.power = getLauncherPower()
            sleep(2000)


            for(i in 1..6) {
                push()
                detectCode()
                launcherMotor.power = getLauncherPower()
                sleep(1000)
            }



        }
    }

    fun push(){
        sleep(200)
        pusher.position = KICK_POS
        sleep(700)
        pusher.position = REST_POS
        sleep(500)
    }

    fun detectCode(){
        val target = getTargetDetection()
        if (target != null) {
            distancePlanar = target.ftcPose.range
        }
        telemetry.addLine("got target $distancePlanar far away")
        telemetry.update()
    }

    fun setHoodAngle(){
        val hoodHoodedness = getHoodHoodedness(); // 0-1
        val angle =  0.04 + (hoodHoodedness - 0.003) * .08
        hoodServoLeft.position = angle
        hoodServoRight.position = angle
    }

    fun getLauncherPower(): Double{
        telemetry.addData("Tag distance", " distance=%f ", distancePlanar)
        val power =
            if((distancePlanar <= 140 && distancePlanar > 103) || distancePlanar < 27){ // do linear approximation if not in bounds we tested to fit polynomial regression
                powerApproximator.approximate(52.0)+0.00189405495879*(distancePlanar-52)
            }
            else if(distancePlanar > 140){
                powerApproximator.approximate(52.0)+0.00201889882271*(distancePlanar-52)

            }
            else powerApproximator.approximate(distancePlanar)
        telemetry.addData("Tag power", " power=%f ", power)
        return power + 0.01
    }

    private fun getTargetDetection(): AprilTagDetection? {
        for (d in aprilTag.detections) {
            if (d.id == TARGET_TAG_ID) return d
        }
        return null
    }

    fun getHoodHoodedness(): Double{
        val normalized = (distancePlanar - 30)/70
        if(normalized > 0.5) return 1.0
        val angledness = (normalized / .5)
        telemetry.addData("Angledness amount", " power=%f ", angledness)
        return angledness
    }

    private fun setManualExposure(exposureMS: Int, gain: Int) {
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
                visionPortal.getCameraControl(ExposureControl::class.java)
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

}