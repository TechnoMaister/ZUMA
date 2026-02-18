package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.SHOOTER_PIDF;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, collector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker;
    public List<Servo> blockers;
    public List<DcMotorEx> motors, shooters, chassis, leftChassis, rightChassis;

    public Hardware(HardwareMap hardwareMap){

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        collector = hardwareMap.get(DcMotorEx.class, "collector");

        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
        rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        leftShoot.setDirection(DcMotorEx.Direction.REVERSE);

        rightBlocker.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear, leftShoot, rightShoot, collector);
        chassis = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        leftChassis = Arrays.asList(leftFront, leftRear);
        rightChassis = Arrays.asList(rightFront, rightRear);
        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx shooter : shooters)
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
    }
}
