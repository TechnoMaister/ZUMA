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
    leftJackUp = .5,
    rightJackUp = .5,
    vMax = 2800,
    minMult = .8,
    midMult = .85,
    maxMult = .92,
    dMid = 80,
    dMax = 110,
    rumblingT = 250,
    blockT = 500,
    collectT = 400,
    backCollectT = 600,
    shootT1 = 250,
    shootT2 = 1000,
    scoreMult1 = .773,
    scoreMult2 = .8,
    scoreMult3 = .9,
    blueX = 12,
    redX = 131,
    goalY = 137,
    slow = .5;

    public static boolean

    team;

    public static

    PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
            20,
            0,
            10,
            12
    );

    public static Pose

    startPoseL = new Pose(56, 8, Math.toRadians(90)),
    startPoseC = new Pose(17.7, 118.8, Math.toRadians(323)),
    shootPoseL = new Pose(56, 11, Math.toRadians(111)),
    shootCtrPose = new Pose(58.3, 58.3),
    shootPoseC = new Pose(46.5, 92.3, Math.toRadians(126)),
    shootPoseC2 = new Pose(34, 103, Math.toRadians(126)),
    collect1Pose = new Pose(10, 35, Math.toRadians(0)),
    collect1CtrPoseL = new Pose(60, 40),
    collect2Pose = new Pose(10, 57, Math.toRadians(0)),
    collect2CtrPoseL = new Pose(72, 72),
    collect2CtrPoseC = new Pose(71.5, 60),
    collect3Pose = new Pose(15.5, 84.5, Math.toRadians(0)),
    collect3CtrPoseL = new Pose(47, 81.6),
    collect3CtrPoseC = new Pose(62, 81),
    parkPose1 = new Pose(20, 70, Math.toRadians(0)),
    parkPose2 = new Pose(56, 27, Math.toRadians(180)),
    robotPose;
}
