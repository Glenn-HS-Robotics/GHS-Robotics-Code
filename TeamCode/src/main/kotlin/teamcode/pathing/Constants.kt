//package teamcode.pathing
//
//import com.pedropathing.follower.Follower
//import com.pedropathing.follower.FollowerConstants
//import com.pedropathing.ftc.FollowerBuilder
//import com.pedropathing.ftc.drivetrains.MecanumConstants
//import com.pedropathing.ftc.localization.Encoder
//import com.pedropathing.ftc.localization.constants.DriveEncoderConstants
//import com.pedropathing.paths.PathConstraints
//import com.qualcomm.robotcore.hardware.DcMotorSimple
//import com.qualcomm.robotcore.hardware.HardwareMap
//
//object Constants {
//    val followerConstants = FollowerConstants()
//        .mass(8.6);
//    val localizerConstants = DriveEncoderConstants()
//        .rightFrontMotorName("front_right")
//        .leftFrontMotorName("front_left")
//        .rightRearMotorName("back_right")
//        .leftRearMotorName("back_left")
//        .leftFrontEncoderDirection(Encoder.REVERSE)
//        .rightFrontEncoderDirection(Encoder.FORWARD)
//        .leftRearEncoderDirection(Encoder.FORWARD)
//        .rightRearEncoderDirection(Encoder.REVERSE)
//        .robotWidth(17.5).robotLength(17.5)
//        .forwardTicksToInches(0.018)
//        .strafeTicksToInches(1.17)
//        .turnTicksToInches(-1.43);
//
//    val driveConstants = MecanumConstants()
//        .maxPower(1.0)
//        .rightFrontMotorName("front_right")
//        .leftFrontMotorName("front_left")
//        .rightRearMotorName("back_right")
//        .leftRearMotorName("back_left")
//        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//
//    val pathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0);
//
//    fun createFollower(hardwareMap: HardwareMap): Follower = FollowerBuilder(followerConstants, hardwareMap)
//        .pathConstraints(pathConstraints)
//        .mecanumDrivetrain(driveConstants)
//        .driveEncoderLocalizer(localizerConstants)
//        .build()
//}