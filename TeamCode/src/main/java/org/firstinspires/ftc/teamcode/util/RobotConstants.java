package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
    collectorPulseT = 200,
    pow = .1,
    tolerance = 250,
    backup = 1,
    offset = 15,
    vMax = 2800,
    errorMin = .6,
    errorMid = .7,
    errorMax = .8,
    dMid = 1.5,
    dMax = 3;

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
