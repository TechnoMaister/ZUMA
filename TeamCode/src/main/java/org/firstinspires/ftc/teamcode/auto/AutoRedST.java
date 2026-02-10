package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.open;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.pickup3Pose;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult2;
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

@Autonomous(name = "Red ST", group = "Auto Red")
public class AutoRedST extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPoseST, scorePoseST, pickupPose, parkPose, goBackPose;
    public Path scorePreload;
    public PathChain park, collect, goBack, goShoot;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPoseST = RobotConstants.startPoseST.mirror();
        scorePoseST = RobotConstants.scorePoseST.mirror();
        pickupPose = RobotConstants.pickup3Pose.mirror();
        goBackPose = RobotConstants.goBackPose.mirror();
        parkPose = RobotConstants.parkPose.mirror();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseST);

        robot = new Hardware(hardwareMap);
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPoseST, scorePoseST));
        scorePreload.setLinearHeadingInterpolation(startPoseST.getHeading(), scorePoseST.getHeading());

        collect = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseST, pickupPose))
                .setLinearHeadingInterpolation(scorePoseST.getHeading(), pickupPose.getHeading())
                .build();

        goBack = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, goBackPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), goBackPose.getHeading())
                .build();

        goShoot = follower.pathBuilder()
                .addPath(new BezierLine(goBackPose, scorePoseST))
                .setLinearHeadingInterpolation(goBackPose.getHeading(), scorePoseST.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseST, parkPose))
                .setLinearHeadingInterpolation(scorePoseST.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                //collect();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //stopCollect();
                    if(actionTimer.getElapsedTime() < scoreT/*+.5*scoreT*/) score();
                    else {
                        stopShoot();
                        block();
                        follower.followPath(collect,true);
                        setPathState(2);
                    }
                } else actionTimer.resetTimer();
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(goBack, true);
                    setPathState(3);
                } else actionTimer.resetTimer();
                break;
            case 3:
                if(!follower.isBusy()) {
                    collectB();
                    follower.followPath(goShoot, true);
                    setPathState(4);
                } else actionTimer.resetTimer();
                break;
            case 4:
                if(!follower.isBusy()) {
                    stopCollect();
                    if(actionTimer.getElapsedTime() < scoreT+.5*scoreT) score();
                    else {
                        stopEverything();
                        setPathState(5);
                    }
                } else actionTimer.resetTimer();
                break;
            case 5:
                follower.followPath(park);
                setPathState(-1);
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

    public void collectB() {
        robot.collector.setVelocity(-0.025*vMax);
    }

    public void stopCollect() {
        robot.collector.setVelocity(0);
    }

    public void score() {
        shoot(scoreMult2);
        if(actionTimer.getElapsedTime() >= shootT) {
            open();
            if(actionTimer.getElapsedTime() >= shootT+open) collect();
        }
    }

    public void stopEverything() {
        stopShoot();
        stopCollect();
        block();
    }
}
