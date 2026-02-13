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
    blockerBlockedPos = .6,
    blockerOpenPos = .38,
    rumblingT = 250,
    blockT = 500,
    collectorPulseT = 100,
    pow = .1,
    tolerance = 250,
    backup = 1,
    offset = 15,
    vMax = 2800,
    errorMin = .62,
    errorMid = .65,
    errorMax = .68,
    dMid = 1,
    dMax = 2,
    scoreT = 2000,
    collectT = 1000,
    shootT = 750,
    open = 250,
    scoreMult1 = .779,
    scoreMult2 = .8;

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

            startPoseS = new Pose(22.3, 125.1, Math.toRadians(142.5)),
            scoreS = new Pose(55.9, 91.3, Math.toRadians(137)),
            interCol1S = new Pose(46.6, 83.1, Math.toRadians(0)),
            col1S = new Pose(16.1, 83.1, Math.toRadians(0)),
            interCol2S = new Pose(46.6, 60.3, Math.toRadians(0)),
            col2S = new Pose(16.1, 60.3, Math.toRadians(0)),
            interCol3S = new Pose(46.6, 35.7, Math.toRadians(0)),
            col3S = new Pose(16.1, 35.7, Math.toRadians(0)),
            parkS = new Pose(20.5, 70.1, Math.toRadians(180)),


    startPoseL = new Pose(56, 8, Math.toRadians(90)),
            scorePoseL = new Pose(61.1, 18.1, Math.toRadians(113)),
            interCol1L = new Pose(47.1, 35.5, Math.toRadians(0)),
            col1L = new Pose(8, 35.5, Math.toRadians(0)),
            interCol2L = new Pose(47.1, 59.7, Math.toRadians(0)),
            col2L = new Pose(7, 59.7, Math.toRadians(0)),
            interCol3L = new Pose(47.1, 83.9, Math.toRadians(0)),
            col3L = new Pose(10, 83.9, Math.toRadians(0)),
            parkL = new Pose(20.5, 70.1, Math.toRadians(180)),

    startPose = new Pose(23.5, 126.5, Math.toRadians(142.5)),
    scorePose = new Pose(60, 95, Math.toRadians(142.5)),
    scorePose1stPickup = new Pose(60, 85, Math.toRadians(142.5)),
    scorePose2 = new Pose(65, 25, Math.toRadians(120)),
    pickup1interPose = new Pose(60, 83, Math.toRadians(0)),
    pickup1Pose = new Pose(23, 83, Math.toRadians(0)),
    pickup2Pose = new Pose(37, 60, Math.toRadians(0)),
    pickup3Pose = new Pose(37, 35, Math.toRadians(0)),
    goBackPose = new Pose(10, 35, Math.toRadians(0)),
    startPoseST = new Pose(60, 8, Math.toRadians(90)),
    scorePoseST = new Pose(60, 12.5, Math.toRadians(112.5)),
    parkPose = new Pose(56, 50, Math.toRadians(180));
}
