package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.SHOOTER_PIDF;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public DcMotorEx collector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker, rightJack, leftJack;;
    public List<Servo> blockers;
    public List<DcMotorEx> motors, shooters;

    public Hardware(HardwareMap hardwareMap){

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
        rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

        leftJack = hardwareMap.get(Servo.class, "leftJack");
        rightJack = hardwareMap.get(Servo.class, "rightJack");

        leftShoot.setDirection(DcMotorEx.Direction.REVERSE);

        rightBlocker.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftShoot, rightShoot, collector);
        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx shooter : shooters)
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
    }
}
