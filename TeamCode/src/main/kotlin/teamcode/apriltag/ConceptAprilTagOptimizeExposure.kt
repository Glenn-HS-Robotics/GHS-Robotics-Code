/* Copyright (c) 2023 FIRST. All rights reserved.
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
package teamcode.apriltag

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.min

/*
 * This OpMode determines the best Exposure for minimizing image motion-blur on a Webcam
 * Note that it is not possible to control the exposure for a Phone Camera, so if you are using a Phone for the Robot Controller
 * this OpMode/Feature only applies to an externally connected Webcam
 *
 * The goal is to determine the smallest (shortest) Exposure value that still provides reliable Tag Detection.
 * Starting with the minimum Exposure and maximum Gain, the exposure is slowly increased until the Tag is
 * detected reliably from the likely operational distance.
 *
 *
 * The best way to run this optimization is to view the camera preview screen while changing the exposure and gains.
 *
 * To do this, you need to view the RobotController screen directly (not from Driver Station)
 * This can be done directly from a RC phone screen (if you are using an external Webcam), but for a Control Hub you must either plug an
 * HDMI monitor into the Control Hub HDMI port, or use an external viewer program like ScrCpy (https://scrcpy.org/)
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "April Tag Optimize Exposer", group = "Concept")
class ConceptAprilTagOptimizeExposure : LinearOpMode() {
    private var visionPortal: VisionPortal? = null // Used to manage the video source.
    private var aprilTag: AprilTagProcessor? =
        null // Used for managing the AprilTag detection process.
    private var myExposure = 0
    private var minExposure = 0
    private var maxExposure = 0
    private var myGain = 0
    private var minGain = 0
    private var maxGain = 0

    var thisExpUp: Boolean = false
    var thisExpDn: Boolean = false
    var thisGainUp: Boolean = false
    var thisGainDn: Boolean = false

    var lastExpUp: Boolean = false
    var lastExpDn: Boolean = false
    var lastGainUp: Boolean = false
    var lastGainDn: Boolean = false
    override fun runOpMode() {
        // Initialize the Apriltag Detection process
        initAprilTag()

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        this.cameraSetting
        myExposure = min(5, minExposure)
        myGain = maxGain
        setManualExposure(myExposure, myGain)

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Touch START to start OpMode")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            telemetry.addLine("Find lowest Exposure that gives reliable detection.")
            telemetry.addLine("Use Left bump/trig to adjust Exposure.")
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n")

            // Display how many Tags Detected
            val currentDetections: MutableList<AprilTagDetection?> = aprilTag!!.getDetections()
            val numTags = currentDetections.size
            if (numTags > 0) telemetry.addData(
                "Tag",
                "####### %d Detected  ######",
                currentDetections.size
            )
            else telemetry.addData("Tag", "----------- none - ----------")

            telemetry.addData("Exposure", "%d  (%d - %d)", myExposure, minExposure, maxExposure)
            telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain)
            telemetry.update()

            // check to see if we need to change exposure or gain.
            thisExpUp = gamepad1.left_bumper
            thisExpDn = gamepad1.left_trigger > 0.25
            thisGainUp = gamepad1.right_bumper
            thisGainDn = gamepad1.right_trigger > 0.25

            // look for clicks to change exposure
            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure)
                setManualExposure(myExposure, myGain)
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure)
                setManualExposure(myExposure, myGain)
            }

            // look for clicks to change the gain
            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain)
                setManualExposure(myExposure, myGain)
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain)
                setManualExposure(myExposure, myGain)
            }

            lastExpUp = thisExpUp
            lastExpDn = thisExpDn
            lastGainUp = thisGainUp
            lastGainDn = thisGainDn

            sleep(20)
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private fun initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = AprilTagProcessor.Builder().build()

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()
    }

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    private fun setManualExposure(exposureMS: Int, gain: Int): Boolean {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false
        }

        // Wait for the camera to be open
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
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            val exposureControl =
                visionPortal!!.getCameraControl<ExposureControl>(ExposureControl::class.java)
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual)
                sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)

            // Set Gain.
            val gainControl = visionPortal!!.getCameraControl<GainControl>(GainControl::class.java)
            gainControl.setGain(gain)
            sleep(20)
            return (true)
        } else {
            return (false)
        }
    }

    private val cameraSetting: Unit
        /*
                Read this camera's minimum and maximum Exposure and Gain settings.
                Can only be called AFTER calling initAprilTag();
             */
        get() {
            // Ensure Vision Portal has been setup.
            if (visionPortal == null) {
                return
            }

            // Wait for the camera to be open
            if (visionPortal!!.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting")
                telemetry.update()
                while (!isStopRequested() && (visionPortal!!.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20)
                }
                telemetry.addData("Camera", "Ready")
                telemetry.update()
            }

            // Get camera control values unless we are stopping.
            if (!isStopRequested()) {
                val exposureControl =
                    visionPortal!!.getCameraControl<ExposureControl>(ExposureControl::class.java)
                minExposure =
                    exposureControl.getMinExposure(TimeUnit.MILLISECONDS)
                        .toInt() + 1
                maxExposure =
                    exposureControl.getMaxExposure(TimeUnit.MILLISECONDS)
                        .toInt()

                val gainControl =
                    visionPortal!!.getCameraControl<GainControl>(
                        GainControl::class.java
                    )
                minGain = gainControl.getMinGain()
                maxGain = gainControl.getMaxGain()
            }
        }
}
