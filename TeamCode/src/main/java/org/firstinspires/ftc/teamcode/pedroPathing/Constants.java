package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration(-41.3)
            .lateralZeroPowerAcceleration(-60.8)
            .translationalPIDFCoefficients(new PIDFCoefficients(.15, 0, .01, .02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, .03, .02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.025, 0, .00001, .6, .01))
            .centripetalScaling(.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .xVelocity(46.5)
            .yVelocity(34.4);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.0005)
            .strafeTicksToInches(.0005)
            .turnTicksToInches(.002)
            .leftPodY(-3.7)
            .rightPodY(3.7)
            .strafePodX(7.52)
            .leftEncoder_HardwareMapName("leftRear")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("leftFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public static PathConstraints pathConstraints = new PathConstraints(1, 100, 2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
