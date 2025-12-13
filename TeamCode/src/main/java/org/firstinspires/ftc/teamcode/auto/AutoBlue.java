package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.open;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@Autonomous(name = "Blue", group = "Auto Blue")
public class AutoBlue extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPose, scorePose, scorePose2, pickup1Pose, pickup1interPose, pickup2Pose, pickup3Pose, scorePose1stPickup;
    public Path scorePreload;
    public PathChain interGrabPickup1, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPose = RobotConstants.startPose;
        scorePose = RobotConstants.scorePose;
        scorePose1stPickup = RobotConstants.scorePose1stPickup;
        scorePose2 = RobotConstants.scorePose2;
        pickup1interPose = RobotConstants.pickup1interPose;
        pickup1Pose = RobotConstants.pickup1Pose;
        pickup2Pose = RobotConstants.pickup2Pose;
        pickup3Pose = RobotConstants.pickup3Pose;

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        robot = new Hardware(hardwareMap);
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        interGrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1interPose))
                .setConstantHeadingInterpolation(pickup1interPose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1interPose, pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1stPickup.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() < scoreT) score();
                    else {
                        stopShoot();
                        block();
                        follower.followPath(interGrabPickup1);
                        follower.followPath(grabPickup1,true);
                        setPathState(2);
                    }
                } else actionTimer.resetTimer();
                break;
            case 2:
                if(!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() > collectT) {
                        stopCollect();
                        follower.followPath(scorePickup1,true);
                        setPathState(3);
                    }
                } else actionTimer.resetTimer();
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(actionTimer.getElapsedTime() < scoreT) score();
                    else {
                        stopShoot();
                        block();
                        setPathState(-1);
                    }
                    //follower.followPath(grabPickup2,true);
                } else actionTimer.resetTimer();
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                end();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("action timer", actionTimer.getElapsedTime());
        telemetry.update();
    }

    public void shoot(double velocity) {
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(velocity * vMax);
    }

    public void stopShoot() {
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
    }

    public void open() {
        for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
    }

    public void block() {
        for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
    }

    public void collect() {
        robot.collector.setVelocity(vMax);
    }

    public void stopCollect() {
        robot.collector.setVelocity(0);
    }

    public void score() {
        shoot(scoreMult1);
        if(actionTimer.getElapsedTime() >= shootT) {
            open();
            if(actionTimer.getElapsedTime() >= shootT+open) collect();
        }
    }

    public void end() {
        follower.turnToDegrees(180);
        stopShoot();
        stopCollect();
        block();
    }
}
