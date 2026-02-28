package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
        public static double collectorReverseMult = -.085,
                        blockerBlockedPos = .6,
                        blockerOpenPos = .38,
                        leftJackUpPos = .65,
                        rightJackUpPos = .55,
                        leftJackDownPos,
                        rightJackDownPos,
                        vMax = 2800,
                        blockT = 500,
                        preShootReverseT = 10,
                        delayT = 250,
                        collectT = 225,
                        shootDelay = 250,
                        shootT1 = 450,
                        reverseT = 1400,
                        shootT2 = 1300,
                        errorX = 2,
                        errorY = 2,
                        errorH = 10,
                        shootC = .714159,
                        shootF = .77,
                        goalY = 135.67,
                        goalX,
                        distance,
                        dx,
                        dy,
                        angleErrorBFR,
                        angleErrorRFR,
                        angleErrorRFL = -6,
                        angleErrorBFL = -8,
                        angleErrorRCR = -4,

                        angleErrorBCR = -4,
                        angleErrorRCL = -16,
                        angleErrorBCL = -4,
                        angleError,
                        shootErrorF = -.04,
                        shootErrorC = 0.02,
                        shootError,
                        limelightOffsetX = -12,
                        limelightAngleOffsetC = 10, // Degrees offset for CLOSE shooting
                        limelightAngleOffsetF = 5.6, // Degrees offset for FAR shooting
                        referenceVoltage = 10; // Voltage reference for close zone compensation

        public static

        PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
                        20,
                        0,
                        10,
                        12),
                        COLLECTOR_PIDF = new PIDFCoefficients(
                                        20,
                                        0,
                                        10,
                                        12);

        public static Pose

        startPoseF = new Pose(56, 8.66, Math.toRadians(90)),
                        startPoseC = new Pose(18.167, 119.267, Math.toRadians(144)),
                        shootPoseF = new Pose(56, 11, Math.toRadians(112.5)),
                        shootPoseCahuit = new Pose(47.69, 85.14159674169, Math.toRadians(130)),
                        shootPoseC = new Pose(50, 88, Math.toRadians(130)),
                        collect1Pose = new Pose(13, 35, Math.toRadians(0)),
                        collect1PoseC = new Pose(12, 35, Math.toRadians(0)),
                        collect1CtrPoseF = new Pose(60, 40),
                        collect1CtrPoseC = new Pose(63, 25.676941),
                        collect2Pose = new Pose(13, 57, Math.toRadians(0)),
                        collect2PoseC = new Pose(8, 57, Math.toRadians(0)),
                        score2CtrPose = new Pose(48, 56),
                        collect2CtrPoseF = new Pose(72, 69.7),
                        collect2CtrPoseC = new Pose(67.6, 55),
                        collect3Pose = new Pose(18, 85, Math.toRadians(0)),
                        collect3PoseC = new Pose(10, 85, Math.toRadians(0)),
                        collect3CtrPoseF = new Pose(47.5, 75),
                        collect3CtrPoseC = new Pose(37.3, 76.967),
                        parkPose1 = new Pose(25, 70, Math.toRadians(0)),
                        parkPoseAhuieala = new Pose(20, 70, Math.toRadians(0)),
                        parkPose2 = new Pose(110, 7.955, Math.toRadians(180)),
                        robotPose;
}
