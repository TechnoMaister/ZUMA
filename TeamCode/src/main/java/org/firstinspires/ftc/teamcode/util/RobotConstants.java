package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static double
    collectorPowerN = 1,
    collectorPowerD = 1,
    collectorPowerB = -.4,
    blockerBlockedPos = .45,
    blockerOpenPos = .6,
    rumblingT = 250,
    blockT = 350,
    k = .1,
    tolerance = 250,
    backup = 1;
    public static
    PDFS_Velocity shooterPID = new PDFS_Velocity(
            0.0007,
            0.0001,
            0.00015
    );
}
