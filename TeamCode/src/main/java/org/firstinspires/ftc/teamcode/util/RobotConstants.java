package org.firstinspires.ftc.teamcode.util;

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
}
