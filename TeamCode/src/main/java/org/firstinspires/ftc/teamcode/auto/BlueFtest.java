package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect1CtrPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect2CtrPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect3CtrPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverseMult;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.distance;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dx;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dy;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.parkPose2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.reverseT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.robotPose;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootDelay;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootPoseC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.startPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@Autonomous(name = "RedFtest", group = "Red")
public class BlueFtest extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer, starlight;
    public Pose
    startPose, shootPose1, shootPose2,
    collect1Pose, collect1CtrPose, collect2Pose,
    collect2CtrPose, collect3Pose, collect3CtrPose, parkPose, score2CtrPose;
    public Path scorePreload;
    public PathChain grab1, score1, grab2, score2, grab3, score3, park1, park2;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPose = startPoseF;
        shootPose1 = shootPoseF;
        shootPose2 = shootPoseC;
        collect1Pose = RobotConstants.collect1Pose;
        collect1CtrPose = collect1CtrPoseF;
        collect2Pose = RobotConstants.collect2Pose;
        collect2CtrPose = collect2CtrPoseF;
        score2CtrPose = RobotConstants.score2CtrPose;
        collect3Pose = RobotConstants.collect3Pose;
        collect3CtrPose = collect3CtrPoseF;
        parkPose = parkPose2;

        goalX = 132;
        gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        starlight = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        robot = new Hardware(hardwareMap);
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, shootPose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());

        grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, collect1CtrPose, collect1Pose))
                .setConstantHeadingInterpolation(collect1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose1))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose1.getHeading())
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, collect2CtrPose, collect2Pose))
                .setConstantHeadingInterpolation(collect2Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(collect2Pose, score2CtrPose, shootPose2))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose2.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, collect3CtrPose, collect3Pose))
                .setConstantHeadingInterpolation(collect3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose2))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose2.getHeading())
                .build();

        park1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, parkPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), parkPose.getHeading())
                .build();

        park2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, parkPose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() >= shootDelay)
                        if (actionTimer.getElapsedTime() <= shootT1) shoot(shootF-.02);
                        else if(actionTimer.getElapsedTime() <= shootT2) {
                            collect();
                            open();
                        } else {
                            stopShoot();
                            block();
                            stopCollect();
                            follower.followPath(grab1, true);
                            setPathState(2);
                        }
                } else actionTimer.resetTimer();

                break;
            case 2:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() >= collectT) {
                        stopCollect();
                        follower.followPath(score1, true);
                        setPathState(3);
                    }
                } else actionTimer.resetTimer();
                break;
            case 3:
                if (!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() >= shootDelay)
                        if (actionTimer.getElapsedTime() <= shootT1) shoot(shootF-0.2);
                        else if(actionTimer.getElapsedTime() <= shootT2) {
                            collect();
                            open();
                        } else {
                            stopShoot();
                            block();
                            follower.followPath(grab2, true);
                            setPathState(4);
                        }
                } else actionTimer.resetTimer();
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() >= collectT) {
                        stopCollect();
                        follower.followPath(score2, true);
                        setPathState(5);
                    }
                } else actionTimer.resetTimer();
                break;
            case 5:
                if (!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() >= shootDelay)
                        if (actionTimer.getElapsedTime() <= shootT1) shoot(shootF-0.2);
                        else if(actionTimer.getElapsedTime() <= shootT2) {
                            collect();
                            open();
                        } else {
                            stopShoot();
                            block();
                            follower.followPath(grab3, true);
                            setPathState(6);
                        }
                } else actionTimer.resetTimer();
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() >= collectT) {
                        stopCollect();
                        follower.followPath(score3, true);
                        setPathState(7);
                    }
                } else actionTimer.resetTimer();
                break;
            case 7:
                if (!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() >= shootDelay)
                        if (actionTimer.getElapsedTime() <= shootT1) shoot(shootF-0.2);
                        else if(actionTimer.getElapsedTime() <= shootT2) {
                            collect();
                            open();
                        } else {
                            stopEverything();
                            follower.followPath(park1, true);
                            setPathState(8);
                        }
                } else actionTimer.resetTimer();
                break;
            case 8:
                if (!follower.isBusy()) setPathState(-1);
                else actionTimer.resetTimer();
                break;
        }
    }
    public void setPathState (int pState){
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        starlight.resetTimer();
    }
    @Override
    public void start () {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop () {
        follower.update();
        autonomousPathUpdate();

        robotPose = follower.getPose();
        dx = goalX - follower.getPose().getX(); dy = goalY - follower.getPose().getY();
        distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("action timer", actionTimer.getElapsedTime());
        telemetry.update();
    }

    public void shoot(double multiplier){
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(multiplier * vMax);
        if(actionTimer.getElapsedTime() >= reverseT) reverseCollect();
    }

    public void stopShoot() {
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
    }

    public void open() {
        for (Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
    }

    public void block() {
        for (Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
    }

    public void collect() {
        robot.collector.setVelocity(vMax);
    }

    public void reverseCollect() {
        robot.collector.setVelocity(collectorReverseMult*vMax);
    }

    public void stopCollect() {
        robot.collector.setVelocity(0);
    }

    public void stopEverything() {
        stopShoot();
        stopCollect();
        block();
    }
}