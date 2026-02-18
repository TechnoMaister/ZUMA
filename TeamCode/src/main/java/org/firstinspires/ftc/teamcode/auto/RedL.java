package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backCollectT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect1CtrPoseL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect2CtrPoseL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collect3CtrPoseL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverse;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.parkPose1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult3;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootPoseC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootPoseC2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootPoseL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.startPoseL;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@Autonomous(name = "RedL", group = "Red")
public class RedL extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose
    startPose, shootPose1, shootPose2, shootPose3,
    collect1Pose, collect1CtrPose, collect2Pose,
    collect2CtrPose, collect3Pose, collect3CtrPose, parkPose;
    public Path scorePreload;
    public PathChain grab1, score1, grab2, score2, grab3, score3, park1, park2;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPose = startPoseL .mirror();
        shootPose1 = shootPoseL.mirror();
        shootPose2 = shootPoseC.mirror();
        shootPose3 = shootPoseC2.mirror();
        collect1Pose = RobotConstants.collect1Pose.mirror();
        collect1CtrPose = collect1CtrPoseL.mirror();
        collect2Pose = RobotConstants.collect2Pose.mirror();
        collect2CtrPose = collect2CtrPoseL.mirror();
        collect3Pose = RobotConstants.collect3Pose.mirror();
        collect3CtrPose = collect3CtrPoseL.mirror();
        parkPose = parkPose1.mirror();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
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
                .addPath(new BezierLine(collect2Pose, shootPose2))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose2.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, collect3CtrPose, collect3Pose))
                .setConstantHeadingInterpolation(collect3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose3))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose3.getHeading())
                .build();

        park1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkPose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), parkPose.getHeading())
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
                    if (actionTimer.getElapsedTime() <= shootT1) score(scoreMult1);
                    else if(actionTimer.getElapsedTime() <= shootT2) {
                        collect();
                        open();
                    } else {
                        stopShoot();
                        block();
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
                    if (actionTimer.getElapsedTime() <= shootT1) score(scoreMult1);
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
                    if (actionTimer.getElapsedTime() <= shootT1) score(scoreMult2);
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
                    if (actionTimer.getElapsedTime() <= shootT1) score(scoreMult3);
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("action timer", actionTimer.getElapsedTime());
        telemetry.update();
    }

    public void shoot(double multiplier){
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(multiplier * vMax);
    }

    public void score(double scoreMult) {
        shoot(scoreMult);
        if(actionTimer.getElapsedTime() >= backCollectT) backCollect();
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

    public void backCollect() {
        robot.collector.setPower(collectorReverse);
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