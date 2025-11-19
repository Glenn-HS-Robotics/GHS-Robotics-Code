package teamcode.pathing

import com.bylazar.configurables.PanelsConfigurables.refreshClass
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.telemetry.SelectScope
import com.pedropathing.telemetry.SelectableOpMode
import com.pedropathing.util.PoseHistory
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.pathing.Drawing.drawRobot
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.Exception
import kotlin.Long
import kotlin.RuntimeException
import kotlin.String
import kotlin.div
import kotlin.math.abs
import kotlin.math.pow

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
class Tuning :
    SelectableOpMode("Select a Tuning OpMode", Consumer { s: SelectScope<Supplier<OpMode?>?>? ->
        s!!.folder("Localization", Consumer { l: SelectScope<Supplier<OpMode?>?>? ->
            l!!.add("Localization Test", Supplier { LocalizationTest() })
            l.add("Forward Tuner", Supplier { ForwardTuner() })
            l.add("Lateral Tuner", Supplier { LateralTuner() })
            l.add("Turn Tuner", Supplier { TurnTuner() })
        })
        s.folder("Automatic", Consumer { a: SelectScope<Supplier<OpMode?>?>? ->
            a!!.add("Forward Velocity Tuner", Supplier { ForwardVelocityTuner() })
            a.add("Lateral Velocity Tuner", Supplier { LateralVelocityTuner() })
            a.add(
                "Forward Zero Power Acceleration Tuner",
                Supplier { ForwardZeroPowerAccelerationTuner() })
            a.add(
                "Lateral Zero Power Acceleration Tuner",
                Supplier { LateralZeroPowerAccelerationTuner() })
        })
        s.folder("Manual", Consumer { p: SelectScope<Supplier<OpMode?>?>? ->
            p!!.add("Translational Tuner", Supplier { TranslationalTuner() })
            p.add("Heading Tuner", Supplier { HeadingTuner() })
            p.add("Drive Tuner", Supplier { DriveTuner() })
            p.add("Centripetal Tuner", Supplier { CentripetalTuner() })
        })
        s.folder("Tests", Consumer { p: SelectScope<Supplier<OpMode?>?>? ->
            p!!.add("Line", Supplier { Line() })
            p.add("Triangle", Supplier { Triangle() })
            p.add("Circle", Supplier { Circle() })
        })
    }) {
    public override fun onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap)
            refreshClass(this)
        } else {
            follower = Constants.createFollower(hardwareMap)
        }

        follower!!.setStartingPose(Pose())

        poseHistory = follower!!.getPoseHistory()

        telemetryM = PanelsTelemetry.telemetry

        Drawing.init()
    }

    public override fun onLog(lines: MutableList<String?>?) {}

    companion object {
        var follower: Follower? = null

        @IgnoreConfigurable
        var poseHistory: PoseHistory? = null

        @IgnoreConfigurable
        var telemetryM: TelemetryManager? = null

        @IgnoreConfigurable
        var changes: ArrayList<String?> = ArrayList<String?>()

        fun drawOnlyCurrent() {
            try {
                drawRobot(follower!!.getPose())
                Drawing.sendPacket()
            } catch (e: Exception) {
                throw RuntimeException("Drawing failed " + e)
            }
        }

        fun draw() {
            Drawing.drawDebug(follower!!)
        }

        /** This creates a full stop of the robot by setting the drive motors to run at 0 power.  */
        fun stopRobot() {
            follower!!.startTeleopDrive(true)
            follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
        }
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
internal class LocalizationTest : OpMode() {
    override fun init() {}

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug(
            "This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1."
        )
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.startTeleopDrive()
        Tuning.Companion.follower!!.update()
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        Tuning.Companion.follower!!.setTeleOpDrive(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
            true
        )
        Tuning.Companion.follower!!.update()

        Tuning.Companion.telemetryM!!.debug("x:" + Tuning.Companion.follower!!.getPose().getX())
        Tuning.Companion.telemetryM!!.debug("y:" + Tuning.Companion.follower!!.getPose().getY())
        Tuning.Companion.telemetryM!!.debug(
            "heading:" + Tuning.Companion.follower!!.getPose().getHeading()
        )
        Tuning.Companion.telemetryM!!.debug("total heading:" + Tuning.Companion.follower!!.getTotalHeading())
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.draw()
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
internal class ForwardTuner : OpMode() {
    override fun init() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()

        Tuning.Companion.telemetryM!!.debug(
            "Distance Moved: " + Tuning.Companion.follower!!.getPose().getX()
        )
        Tuning.Companion.telemetryM!!.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.")
        Tuning.Companion.telemetryM!!.debug(
            "Multiplier: " + (DISTANCE / (Tuning.Companion.follower!!.getPose()
                .getX() / Tuning.Companion.follower!!.getPoseTracker().getLocalizer()
                .getForwardMultiplier()))
        )
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.draw()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
internal class LateralTuner : OpMode() {
    override fun init() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()

        Tuning.Companion.telemetryM!!.debug(
            "Distance Moved: " + Tuning.Companion.follower!!.getPose().getY()
        )
        Tuning.Companion.telemetryM!!.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.")
        Tuning.Companion.telemetryM!!.debug(
            "Multiplier: " + (DISTANCE / (Tuning.Companion.follower!!.getPose()
                .getY() / Tuning.Companion.follower!!.getPoseTracker().getLocalizer()
                .getLateralMultiplier()))
        )
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.draw()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
internal class TurnTuner : OpMode() {
    override fun init() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.")
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()

        Tuning.Companion.telemetryM!!.debug("Total Angle: " + Tuning.Companion.follower!!.getTotalHeading())
        Tuning.Companion.telemetryM!!.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.")
        Tuning.Companion.telemetryM!!.debug(
            "Multiplier: " + (ANGLE / (Tuning.Companion.follower!!.getTotalHeading() / Tuning.Companion.follower!!.getPoseTracker()
                .getLocalizer().getTurningMultiplier()))
        )
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.draw()
    }

    companion object {
        var ANGLE: Double = 2 * Math.PI
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class ForwardVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room, since the robot has inertia after cutting power.")
        Tuning.Companion.telemetryM!!.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.")
        Tuning.Companion.telemetryM!!.debug("Press B on game pad 1 to stop.")
        Tuning.Companion.telemetryM!!.debug("pose", Tuning.Companion.follower!!.getPose())
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        Tuning.Companion.follower!!.startTeleopDrive(true)
        Tuning.Companion.follower!!.update()
        end = false
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.Companion.stopRobot()
            requestOpModeStop()
        }

        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()


        if (!end) {
            if (abs(Tuning.Companion.follower!!.getPose().getX()) > DISTANCE) {
                end = true
                Tuning.Companion.stopRobot()
            } else {
                Tuning.Companion.follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
                val currentVelocity: Double =
                    abs(Tuning.Companion.follower!!.poseTracker.getLocalizer().getVelocity().getX())
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            Tuning.Companion.stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()
            Tuning.Companion.telemetryM!!.debug("Forward Velocity: " + average)
            Tuning.Companion.telemetryM!!.debug("\n")
            Tuning.Companion.telemetryM!!.debug("Press A to set the Forward Velocity temporarily (while robot remains on).")

            for (i in velocities.indices) {
                telemetry.addData(i.toString(), velocities.get(i))
            }

            Tuning.Companion.telemetryM!!.update(telemetry)
            telemetry.update()

            if (gamepad1.aWasPressed()) {
                Tuning.Companion.follower!!.setXVelocity(average)
                val message = "XMovement: " + average
                Tuning.Companion.changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class LateralVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()

    private var end = false

    override fun init() {}

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * telemetryM.
     */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room, since the robot has inertia after cutting power.")
        Tuning.Companion.telemetryM!!.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.")
        Tuning.Companion.telemetryM!!.debug("Press B on Gamepad 1 to stop.")
        Tuning.Companion.telemetryM!!.update(telemetry)

        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run right at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        Tuning.Companion.follower!!.startTeleopDrive(true)
        Tuning.Companion.follower!!.update()
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.Companion.stopRobot()
            requestOpModeStop()
        }

        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (!end) {
            if (abs(Tuning.Companion.follower!!.getPose().getY()) > DISTANCE) {
                end = true
                Tuning.Companion.stopRobot()
            } else {
                Tuning.Companion.follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
                val currentVelocity: Double = abs(
                    Tuning.Companion.follower!!.getVelocity().dot(
                        Vector(1.0, Math.PI / 2)
                    )
                )
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            Tuning.Companion.stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()

            Tuning.Companion.telemetryM!!.debug("Strafe Velocity: " + average)
            Tuning.Companion.telemetryM!!.debug("\n")
            Tuning.Companion.telemetryM!!.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).")
            Tuning.Companion.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.Companion.follower!!.setYVelocity(average)
                val message = "YMovement: " + average
                Tuning.Companion.changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
internal class ForwardZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0

    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetryM.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.")
        Tuning.Companion.telemetryM!!.debug("Then, it will cut power from the drivetrain and roll to a stop.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.Companion.telemetryM!!.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.")
        Tuning.Companion.telemetryM!!.debug("Press B on Gamepad 1 to stop.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        Tuning.Companion.follower!!.startTeleopDrive(false)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.Companion.stopRobot()
            requestOpModeStop()
        }

        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        val heading = Vector(1.0, Tuning.Companion.follower!!.getPose().getHeading())
        if (!end) {
            if (!stopping) {
                if (Tuning.Companion.follower!!.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = Tuning.Companion.follower!!.getVelocity().dot(heading)
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    Tuning.Companion.follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = Tuning.Companion.follower!!.getVelocity().dot(heading)
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < Tuning.Companion.follower!!.getConstraints()
                        .getVelocityConstraint()
                ) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            Tuning.Companion.telemetryM!!.debug("Forward Zero Power Acceleration (Deceleration): " + average)
            Tuning.Companion.telemetryM!!.debug("\n")
            Tuning.Companion.telemetryM!!.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).")
            Tuning.Companion.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.Companion.follower!!.getConstants().setForwardZeroPowerAcceleration(average)
                val message = "Forward Zero Power Acceleration: " + average
                Tuning.Companion.changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
internal class LateralZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0
    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetry.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("The robot will run to the right until it reaches " + VELOCITY + " inches per second.")
        Tuning.Companion.telemetryM!!.debug("Then, it will cut power from the drivetrain and roll to a stop.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.Companion.telemetryM!!.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.")
        Tuning.Companion.telemetryM!!.debug("Press B on game pad 1 to stop.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        Tuning.Companion.follower!!.startTeleopDrive(false)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            Tuning.Companion.stopRobot()
            requestOpModeStop()
        }

        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        val heading = Vector(1.0, Tuning.Companion.follower!!.getPose().getHeading() - Math.PI / 2)
        if (!end) {
            if (!stopping) {
                if (abs(Tuning.Companion.follower!!.getVelocity().dot(heading)) > VELOCITY) {
                    previousVelocity = abs(Tuning.Companion.follower!!.getVelocity().dot(heading))
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    Tuning.Companion.follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double =
                    abs(Tuning.Companion.follower!!.getVelocity().dot(heading))
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < Tuning.Companion.follower!!.getConstraints()
                        .getVelocityConstraint()
                ) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            Tuning.Companion.telemetryM!!.debug("Lateral Zero Power Acceleration (Deceleration): " + average)
            Tuning.Companion.telemetryM!!.debug("\n")
            Tuning.Companion.telemetryM!!.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).")
            Tuning.Companion.telemetryM!!.update(telemetry)

            if (gamepad1.aWasPressed()) {
                Tuning.Companion.follower!!.getConstants().setLateralZeroPowerAcceleration(average)
                val message = "Lateral Zero Power Acceleration: " + average
                Tuning.Companion.changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class TranslationalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will activate the translational PIDF(s)")
        Tuning.Companion.telemetryM!!.debug("The robot will try to stay in place while you push it laterally.")
        Tuning.Companion.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.deactivateAllPIDFs()
        Tuning.Companion.follower!!.activateTranslational()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.Companion.follower!!.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (!Tuning.Companion.follower!!.isBusy()) {
            if (forward) {
                forward = false
                Tuning.Companion.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.Companion.follower!!.followPath(forwards)
            }
        }

        Tuning.Companion.telemetryM!!.debug("Push the robot laterally to test the Translational PIDF(s).")
        Tuning.Companion.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class HeadingTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will activate the heading PIDF(s).")
        Tuning.Companion.telemetryM!!.debug("The robot will try to stay at a constant heading while you try to turn it.")
        Tuning.Companion.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.deactivateAllPIDFs()
        Tuning.Companion.follower!!.activateHeading()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.Companion.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (!Tuning.Companion.follower!!.isBusy()) {
            if (forward) {
                forward = false
                Tuning.Companion.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.Companion.follower!!.followPath(forwards)
            }
        }

        Tuning.Companion.telemetryM!!.debug("Turn the robot manually to test the Heading PIDF(s).")
        Tuning.Companion.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class DriveTuner : OpMode() {
    private var forward = true

    private var forwards: PathChain? = null
    private var backwards: PathChain? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.")
        Tuning.Companion.telemetryM!!.debug("The robot will go forward and backward continuously along the path.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.deactivateAllPIDFs()
        Tuning.Companion.follower!!.activateDrive()

        forwards = Tuning.Companion.follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        backwards = Tuning.Companion.follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        Tuning.Companion.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (!Tuning.Companion.follower!!.isBusy()) {
            if (forward) {
                forward = false
                Tuning.Companion.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.Companion.follower!!.followPath(forwards)
            }
        }

        Tuning.Companion.telemetryM!!.debug("Driving forward?: " + forward)
        Tuning.Companion.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Line : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will activate all the PIDF(s)")
        Tuning.Companion.telemetryM!!.debug("The robot will go forward and backward continuously along the path while correcting.")
        Tuning.Companion.telemetryM!!.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.activateAllPIDFs()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        Tuning.Companion.follower!!.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (!Tuning.Companion.follower!!.isBusy()) {
            if (forward) {
                forward = false
                Tuning.Companion.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.Companion.follower!!.followPath(forwards)
            }
        }

        Tuning.Companion.telemetryM!!.debug("Driving Forward?: " + forward)
        Tuning.Companion.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
internal class CentripetalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.")
        Tuning.Companion.telemetryM!!.debug("The robot will go continuously along the path.")
        Tuning.Companion.telemetryM!!.debug("Make sure you have enough room.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun start() {
        Tuning.Companion.follower!!.activateAllPIDFs()
        forwards =
            Path(BezierCurve(Pose(), Pose(abs(DISTANCE), 0.0), Pose(abs(DISTANCE), DISTANCE)))
        backwards = Path(
            BezierCurve(
                Pose(abs(DISTANCE), DISTANCE),
                Pose(abs(DISTANCE), 0.0),
                Pose(0.0, 0.0)
            )
        )

        backwards!!.setTangentHeadingInterpolation()
        backwards!!.reverseHeadingInterpolation()

        Tuning.Companion.follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()
        if (!Tuning.Companion.follower!!.isBusy()) {
            if (forward) {
                forward = false
                Tuning.Companion.follower!!.followPath(backwards)
            } else {
                forward = true
                Tuning.Companion.follower!!.followPath(forwards)
            }
        }

        Tuning.Companion.telemetryM!!.debug("Driving away from the origin along the curve?: " + forward)
        Tuning.Companion.telemetryM!!.update(telemetry)
    }

    companion object {
        var DISTANCE: Double = 20.0
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
internal class Triangle : OpMode() {
    private val startPose = Pose(0.0, 0.0, Math.toRadians(0.0))
    private val interPose = Pose(24.0, -24.0, Math.toRadians(90.0))
    private val endPose = Pose(24.0, 24.0, Math.toRadians(45.0))

    private var triangle: PathChain? = null

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (Tuning.Companion.follower!!.atParametricEnd()) {
            Tuning.Companion.follower!!.followPath(triangle, true)
        }
    }

    override fun init() {}

    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.")
        Tuning.Companion.telemetryM!!.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    /** Creates the PathChain for the "triangle". */
    override fun start() {
        Tuning.Companion.follower!!.setStartingPose(startPose)

        triangle = Tuning.Companion.follower!!.pathBuilder()
            .addPath(BezierLine(startPose, interPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
            .addPath(BezierLine(interPose, endPose))
            .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
            .addPath(BezierLine(endPose, startPose))
            .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
            .build()

        Tuning.Companion.follower!!.followPath(triangle)
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Circle : OpMode() {
    private var circle: PathChain? = null

    override fun start() {
        circle = Tuning.Companion.follower!!.pathBuilder()
            .addPath(BezierCurve(Pose(0.0, 0.0), Pose(RADIUS, 0.0), Pose(RADIUS, RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(
                BezierCurve(
                    Pose(RADIUS, RADIUS),
                    Pose(RADIUS, 2 * RADIUS),
                    Pose(0.0, 2 * RADIUS)
                )
            )
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(
                BezierCurve(
                    Pose(0.0, 2 * RADIUS),
                    Pose(-RADIUS, 2 * RADIUS),
                    Pose(-RADIUS, RADIUS)
                )
            )
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(-RADIUS, RADIUS), Pose(-RADIUS, 0.0), Pose(0.0, 0.0)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .build()
        Tuning.Companion.follower!!.followPath(circle)
    }

    override fun init_loop() {
        Tuning.Companion.telemetryM!!.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ")
        Tuning.Companion.telemetryM!!.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.")
        Tuning.Companion.telemetryM!!.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.")
        Tuning.Companion.telemetryM!!.update(telemetry)
        Tuning.Companion.follower!!.update()
        Tuning.Companion.drawOnlyCurrent()
    }

    override fun init() {}

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        Tuning.Companion.follower!!.update()
        Tuning.Companion.draw()

        if (Tuning.Companion.follower!!.atParametricEnd()) {
            Tuning.Companion.follower!!.followPath(circle)
        }
    }

    companion object {
        var RADIUS: Double = 10.0
    }
}

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
internal object Drawing {
    val ROBOT_RADIUS: Double = 9.0 // woah
    private val panelsField = field

    private val robotLook = Style(
        "", "#3F51B5", 0.75
    )
    private val historyLook = Style(
        "", "#4CAF50", 0.75
    )

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    fun init() {
        panelsField.setOffsets(presets.PEDRO_PATHING)
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    fun drawDebug(follower: Follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook)
            val closestPoint =
                follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue())
            drawRobot(
                Pose(
                    closestPoint.getX(),
                    closestPoint.getY(),
                    follower.getCurrentPath()
                        .getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())
                ), robotLook
            )
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook)
        drawRobot(follower.getPose(), historyLook)

        sendPacket()
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    @JvmOverloads
    fun drawRobot(pose: Pose?, style: Style = robotLook) {
        if (pose == null || Double.NaN == pose.getX() || Double.NaN == (pose.getY()) || Double.NaN == pose.getHeading())
         {
            return
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.getX(), pose.getY())
        panelsField.circle(ROBOT_RADIUS)

        val v = pose.getHeadingAsUnitVector()
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS)
        val x1 = pose.getX() + v.getXComponent() / 2
        val y1 = pose.getY() + v.getYComponent() / 2
        val x2 = pose.getX() + v.getXComponent()
        val y2 = pose.getY() + v.getYComponent()

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    fun drawPath(path: Path, style: Style) {
        val points = path.getPanelsDrawingPoints()

        for (i in points[0]!!.indices) {
            for (j in points.indices) {
                if (Double.NaN == (points[j]!![i])) {
                    points[j]!![i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(points[0]!![0], points[0]!![1])
        panelsField.line(points[1]!![0], points[1]!![1])
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    fun drawPath(pathChain: PathChain, style: Style) {
        for (i in 0..<pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    @JvmOverloads
    fun drawPoseHistory(poseTracker: PoseHistory, style: Style = historyLook) {
        panelsField.setStyle(style)

        val size = poseTracker.getXPositionsArray().size
        for (i in 0..<size - 1) {
            panelsField.moveCursor(
                poseTracker.getXPositionsArray()[i],
                poseTracker.getYPositionsArray()[i]
            )
            panelsField.line(
                poseTracker.getXPositionsArray()[i + 1],
                poseTracker.getYPositionsArray()[i + 1]
            )
        }
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    fun sendPacket() {
        panelsField.update()
    }
}
