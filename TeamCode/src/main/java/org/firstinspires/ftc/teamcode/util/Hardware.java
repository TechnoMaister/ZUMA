package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.COLLECTOR_PIDF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.SHOOTER_PIDF;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, collector, leftShoot, rightShoot;
    public Servo leftBlocker, rightBlocker;
    public List<Servo> blockers;
    public List<DcMotorEx> shooters;
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

        shooters = Arrays.asList(leftShoot, rightShoot);
        blockers = Arrays.asList(leftBlocker, rightBlocker);

        for (DcMotorEx shooter : shooters) {
            shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        }

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, COLLECTOR_PIDF);
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
