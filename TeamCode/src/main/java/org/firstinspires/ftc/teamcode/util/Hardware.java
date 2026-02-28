package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.COLLECTOR_PIDF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.SHOOTER_PIDF;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public DcMotorEx collector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker, leftJack, rightJack;;
    public List<Servo> blockers;
    public List<DcMotorEx> motors, shooters;
    public AprilTagLimelight limelight;
    public VoltageSensor batteryVoltageSensor;

    public Hardware(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        limelight = new AprilTagLimelight(hardwareMap);

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
        rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

        leftJack = hardwareMap.get(Servo.class, "leftJack");
        rightJack = hardwareMap.get(Servo.class, "rightJack");

        leftShoot.setDirection(DcMotorEx.Direction.REVERSE);

        rightBlocker.setDirection(Servo.Direction.REVERSE);

        rightJack.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftShoot, rightShoot, collector);
        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx shooter : shooters)
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);

        collector.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, COLLECTOR_PIDF);
    }

    public LLResultTypes.FiducialResult getTagBySpecificID(int id) {
        LLResult res = limelight.getLatestResult();
        if (res != null && res.isValid()) {
            for (LLResultTypes.FiducialResult f : res.getFiducialResults()) {
                if (f.getFiducialId() == id)
                    return f;
            }
        }
        return null;
    }
}
