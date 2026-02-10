package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.open;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.scoreMult1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "AutoLongBlue12", group = "Auto Blue")
public class AutoLBlue12ARTEFACTS extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPoseL, scorePoseL, interCol1L, col1L, interCol2L, col2L, interCol3L, col3L, parkL;
    public Path scorePreloadL;
    public PathChain interGrabPickup1L, grabPickup1L, scorePickup1L, interGrabPickup2L, grabPickup2L, scorePickup2L, interGrabPickup3L, grabPickup3L, scorePickup3L, parkingL;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPoseL = RobotConstants.startPoseL;
        scorePoseL = RobotConstants.scorePoseL;
        interCol1L = RobotConstants.interCol1L;
        col1L = RobotConstants.col1L;
        interCol2L = RobotConstants.interCol2L;
        col2L = RobotConstants.col2L;
        interCol3L = RobotConstants.interCol3L;
        col3L = RobotConstants.col3L;
        parkL = RobotConstants.parkL;

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseL);
        robot = new Hardware(hardwareMap);
    }

    public void buildPaths() {
        scorePreloadL = new Path(new BezierLine(startPoseL, scorePoseL));
        scorePreloadL.setLinearHeadingInterpolation(startPoseL.getHeading(), scorePoseL.getHeading());

        interGrabPickup1L = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseL, interCol1L))
                .setLinearHeadingInterpolation(scorePoseL.getHeading(), interCol1L.getHeading())
                .build();

        grabPickup1L = follower.pathBuilder()
                .addPath(new BezierLine(interCol1L, col1L))
                .setConstantHeadingInterpolation(interCol1L.getHeading())
                .build();

        scorePickup1L = follower.pathBuilder()
                .addPath(new BezierLine(col1L, scorePoseL))
                .setLinearHeadingInterpolation(col1L.getHeading(), scorePoseL.getHeading())
                .build();

        interGrabPickup2L = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseL, interCol2L))
                .setLinearHeadingInterpolation(scorePoseL.getHeading(), interCol2L.getHeading())
                .build();

        grabPickup2L = follower.pathBuilder()
                .addPath(new BezierLine(interCol2L, col2L))
                .setConstantHeadingInterpolation(interCol2L.getHeading())
                .build();

        scorePickup2L = follower.pathBuilder()
                .addPath(new BezierLine(col2L, scorePoseL))
                .setLinearHeadingInterpolation(col2L.getHeading(), scorePoseL.getHeading())
                .build();

        interGrabPickup3L = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseL, interCol3L))
                .setLinearHeadingInterpolation(scorePoseL.getHeading(), interCol3L.getHeading())
                .build();

        grabPickup3L = follower.pathBuilder()
                .addPath(new BezierLine(interCol3L, col3L))
                .setConstantHeadingInterpolation(interCol3L.getHeading())
                .build();

        scorePickup3L = follower.pathBuilder()
                .addPath(new BezierLine(col3L, scorePoseL))
                .setLinearHeadingInterpolation(col3L.getHeading(), scorePoseL.getHeading())
                .build();

        parkingL = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseL, parkL))
                .setLinearHeadingInterpolation(scorePoseL.getHeading(), parkL.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadL);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        block();
                        follower.followPath(interGrabPickup1L);
                        follower.followPath(grabPickup1L, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup1L, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        follower.followPath(interGrabPickup2L, true);
                        follower.followPath(grabPickup2L, true);
                        setPathState(4);

                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup2L, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy())
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        follower.followPath(interGrabPickup3L, true);
                        follower.followPath(grabPickup3L, true);
                        setPathState(6);
                    }
                break;
            case 6:
                if (!follower.isBusy())
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup3L, true);
                        setPathState(7);
                    }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        follower.followPath(parkingL, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;


        }
    }
    public void setPathState ( int pState){
        pathState = pState;
        pathTimer.resetTimer();
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

    public void shoot ( double velocity){
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(velocity * vMax);
    }

    public void stopShoot () {
        for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
    }

    public void open () {
        for (Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
    }

    public void block () {
        for (Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
    }

    public void collect () {
        robot.collector.setVelocity(vMax);
    }
    public void stopCollect () {
        robot.collector.setVelocity(0);
    }

    public void score () {
        shoot(scoreMult1);
        if (actionTimer.getElapsedTime() >= shootT) {
            open();
            if (actionTimer.getElapsedTime() >= shootT + open) collect();
        }
    }

    public void stopEverything () {
        stopShoot();
        stopCollect();
        block();
    }

}