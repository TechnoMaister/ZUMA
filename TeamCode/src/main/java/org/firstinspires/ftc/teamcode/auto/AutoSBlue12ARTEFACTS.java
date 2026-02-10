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

@Autonomous(name = "AutoSBlue12", group = "Auto Blue")
public class AutoSBlue12ARTEFACTS extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPoseS, scoreS, interCol1S, col1S, interCol2S, col2S, interCol3S, col3S, parkS;
    public Path scorePreloadS;
    public PathChain interGrabPickup1S, grabPickup1S, scorePickup1S, interGrabPickup2S, grabPickup2S, scorePickup2S, interGrabPickup3S, grabPickup3S, scorePickup3S, parkingS;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPoseS = RobotConstants.startPoseS;
        scoreS = RobotConstants.scoreS;
        interCol1S = RobotConstants.interCol1S;
        col1S = RobotConstants.col1S;
        interCol2S = RobotConstants.interCol2S;
        col2S = RobotConstants.col2S;
        interCol3S = RobotConstants.interCol3S;
        col3S = RobotConstants.col3S;
        parkS = RobotConstants.parkS;

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseS);

        robot = new Hardware(hardwareMap);
    }

    public void buildPaths() {
        scorePreloadS = new Path(new BezierLine(startPoseS, scoreS));
        scorePreloadS.setLinearHeadingInterpolation(startPoseS.getHeading(), scoreS.getHeading());

        interGrabPickup1S = follower.pathBuilder()
                .addPath(new BezierLine(scoreS, interCol1S))
                .setLinearHeadingInterpolation(scoreS.getHeading(), interCol1S.getHeading())
                .build();

        grabPickup1S = follower.pathBuilder()
                .addPath(new BezierLine(interCol1S, col1S))
                .setConstantHeadingInterpolation(interCol1S.getHeading())
                .build();

        scorePickup1S = follower.pathBuilder()
                .addPath(new BezierLine(col1S, scoreS))
                .setLinearHeadingInterpolation(col1S.getHeading(), scoreS.getHeading())
                .build();

        interGrabPickup2S = follower.pathBuilder()
                .addPath(new BezierLine(scoreS, interCol2S))
                .setLinearHeadingInterpolation(scoreS.getHeading(), interCol2S.getHeading())
                .build();

        grabPickup2S = follower.pathBuilder()
                .addPath(new BezierLine(interCol2S, col2S))
                .setConstantHeadingInterpolation(interCol2S.getHeading())
                .build();

        scorePickup2S = follower.pathBuilder()
                .addPath(new BezierLine(col2S, scoreS))
                .setLinearHeadingInterpolation(col2S.getHeading(), scoreS.getHeading())
                .build();

        interGrabPickup3S = follower.pathBuilder()
                .addPath(new BezierLine(scoreS, interCol3S))
                .setLinearHeadingInterpolation(scoreS.getHeading(), interCol3S.getHeading())
                .build();

        grabPickup3S = follower.pathBuilder()
                .addPath(new BezierLine(interCol3S, col3S))
                .setConstantHeadingInterpolation(interCol3S.getHeading())
                .build();

        scorePickup3S = follower.pathBuilder()
                .addPath(new BezierLine(col3S, scoreS))
                .setLinearHeadingInterpolation(col3S.getHeading(), scoreS.getHeading())
                .build();

        parkingS = follower.pathBuilder()
                .addPath(new BezierLine(scoreS, parkS))
                .setLinearHeadingInterpolation(scoreS.getHeading(), parkS.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreloadS);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        block();
                        follower.followPath(interGrabPickup1S);
                        follower.followPath(grabPickup1S, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup1S, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        follower.followPath(interGrabPickup2S, true);
                        follower.followPath(grabPickup2S, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup2S, true);
                        setPathState(5);
                    }

                }
                break;
            case 5:
                if (!follower.isBusy())
                    if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                    else {
                        stopShoot();
                        follower.followPath(interGrabPickup3S, true);
                        follower.followPath(grabPickup3S, true);
                        setPathState(6);
                    }
                break;
            case 6:
                if (!follower.isBusy())
                    if (actionTimer.getElapsedTime() > RobotConstants.collectT) {
                        stopCollect();
                        follower.followPath(scorePickup3S, true);
                        setPathState(7);
                    }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (!follower.isBusy()) {
                        if (actionTimer.getElapsedTime() < RobotConstants.scoreT) score();
                        else {
                            stopEverything();
                            follower.followPath(parkingS, true);
                            setPathState(8);
                        }
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

    public void stopEverything(){
        stopShoot();
        stopCollect();
        block();
    }

}