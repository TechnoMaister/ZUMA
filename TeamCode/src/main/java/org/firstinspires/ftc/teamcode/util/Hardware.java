package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public AprilTagWebcam aprilTagWebcam;
    public IMU imu;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker;
    public List<Servo> blockers;
    public List<DcMotorEx> motors, chassis, shooters;

    public Hardware(HardwareMap hardwareMap){
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap);

        imu=hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters=new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");

        colector = hardwareMap.get(DcMotorEx.class, "colector");
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
        rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBlocker.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot);
        chassis = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
