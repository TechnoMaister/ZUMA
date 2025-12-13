package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static double

    collector_multiplierN = 1,
    collector_multiplierD = 1,
    collector_multiplierB = -.5,
    blockerBlockedPos = .45,
    blockerOpenPos = .6,
    rumblingT = 250,
    blockT = 500,
    collectorPulseT = 100,
    pow = .1,
    tolerance = 250,
    backup = 1,
    offset = 15,
    vMax = 2800,
    errorMin = .6,
    errorMid = .63,
    errorMax = .65,
    dMid = 1,
    dMax = 2,
    scoreT = 2000,
    collectT = 1000,
    shootT = 750,
    open = 250,
    scoreMult1 = .7;

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

    startPose = new Pose(23.5, 126.5, Math.toRadians(142.5)),
    scorePose = new Pose(60, 95, Math.toRadians(142.5)),
    scorePose1stPickup = new Pose(60, 85, Math.toRadians(142.5)),
    scorePose2 = new Pose(65, 25, Math.toRadians(120)),
    pickup1interPose = new Pose(60, 83, Math.toRadians(0)),
    pickup1Pose = new Pose(23, 83, Math.toRadians(0)),
    pickup2Pose = new Pose(37, 60, Math.toRadians(0)),
    pickup3Pose = new Pose(37, 35, Math.toRadians(0)),
    startPoseST = new Pose(84, 8, Math.toRadians(90)),
    scorePoseST = new Pose(84, 12.5, Math.toRadians(121)),
    parkPose = new Pose(38, 33.5, Math.toRadians(0)),
    parkPoseControlPoint = new Pose(121.5, 68.5);
}
