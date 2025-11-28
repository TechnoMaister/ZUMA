package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Hardware {

    public IMU imu;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, collector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker;
    public List<Servo> blockers;
    public List<DcMotorEx> motors, chassis, shooters;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();

    public Hardware(HardwareMap hardwareMap){
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

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

        collector = hardwareMap.get(DcMotorEx.class, "collector");
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
        rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBlocker.setDirection(Servo.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear, collector, leftShoot, rightShoot);
        chassis = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        for(AprilTagDetection detection : detectedTags)
            if(detection.id == id) return detection;
        return null;
    }
}
