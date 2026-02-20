package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static double
    collectorReverse = -.1,
    blockerBlockedPos = .6,
    blockerOpenPos = .38,
    leftJackUp = .6,
    rightJackUp = .5,
    leftJackDown = .25,
    rightJackDown = .75,
    vMax = 2800,
    rumblingT = 250,
    blockT = 500,
    collectT = 225,
    shootT1 = 350,
    shootT2 = 1300,
    errorX = 2,
    errorY = 2,
    errorH = 10,
    errorC = -.02,
    goalY = 137,
    goalX,
    distance,
    dx,
    dy;

    public static

    PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
            20,
            0,
            10,
            12
    );

    public static Pose

    startPoseL = new Pose(56, 8, Math.toRadians(90)),
    startPoseC = new Pose(17.7, 118.8, Math.toRadians(143)),
    shootPoseL = new Pose(56, 11, Math.toRadians(111)),
    shootPoseLC = new Pose(56, 15, Math.toRadians(113.5)),
    shootCtrPose = new Pose(58.3, 58.3),
    shootPoseC = new Pose(50, 88, Math.toRadians(130)),
    collect1Pose = new Pose(13, 35, Math.toRadians(0)),
    collect1CtrPoseL = new Pose(60, 40),
    collect2Pose = new Pose(13, 57, Math.toRadians(0)),
    collect2CtrPoseL = new Pose(72, 72),
    collect2CtrPoseC = new Pose(67.6, 55),
    collect3Pose = new Pose(18, 85, Math.toRadians(0)),
    collect3CtrPoseL = new Pose(47.5, 75),
    collect3CtrPoseC = new Pose(37.3, 79),
    score2CtrPose = new Pose(48, 56),
    parkPose1 = new Pose(25, 70, Math.toRadians(0)),
    parkPose2 = new Pose(56, 27, Math.toRadians(180)),
    robotPose;
}
