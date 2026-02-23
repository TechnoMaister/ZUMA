package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static double
    collectorReverseMult = -.1,
    blockerBlockedPos = .6,
    blockerOpenPos = .38,
    leftJackUpPos = .65,
    rightJackUpPos = .55,
    leftJackDownPos,
    rightJackDownPos,
    vMax = 2800,
    rumblingT = 250,
    blockT = 500,
    collectT = 225,
    shootDelay = 250,
    shootT1 = 450,
    shootT2 = 1300,
    errorX = 2,
    errorY = 2,
    errorH = 10,
    shootC = 725,
    shootF = .77,
    goalY = 135.67,
    goalX,
    distance,
    dx,
    dy,
    angleErrorF,
    angleErrorC,
    angleError,
    shootErrorF,
    shootErrorC,
    shootError;

    public static

    PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
            20,
            0,
            10,
            12
    ),
    COLLECTOR_PIDF = new PIDFCoefficients(
            20,
            0,
            10,
            12
    );

    public static Pose

    startPoseF = new Pose(56, 8.66, Math.toRadians(90)),
    startPoseC = new Pose(18.167, 119.267, Math.toRadians(144)),
    shootPoseF = new Pose(56, 11, Math.toRadians(111)),
    shootPoseC = new Pose(50, 88, Math.toRadians(130)),
    collect1Pose = new Pose(13, 35, Math.toRadians(0)),
    collect1CtrPoseF = new Pose(60, 40),
    collect1CtrPoseC = new Pose(63, 28),
    collect2Pose = new Pose(13, 57, Math.toRadians(0)),
    score2CtrPose = new Pose(48, 56),
    collect2CtrPoseF = new Pose(72, 72),
    collect2CtrPoseC = new Pose(67.6, 55),
    collect3Pose = new Pose(18, 85, Math.toRadians(0)),
    collect3CtrPoseF = new Pose(47.5, 75),
    collect3CtrPoseC = new Pose(37.3, 79),
    parkPose1 = new Pose(25, 70, Math.toRadians(0)),
    parkPose2 = new Pose(56, 27, Math.toRadians(180)),
    robotPose;
}
