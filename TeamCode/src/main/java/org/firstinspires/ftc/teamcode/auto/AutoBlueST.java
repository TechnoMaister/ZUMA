package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
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

@Autonomous(name = "Blue ST", group = "Auto Blue")
public class AutoBlueST extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPoseST, scorePoseST, parkPose;
    public Path scorePreload;
    public PathChain park;
    public Hardware robot;
    public int pathState;

    @Override
    public void init() {
        startPoseST = RobotConstants.startPoseST;
        scorePoseST = RobotConstants.scorePoseST;
        parkPose = RobotConstants.parkPose;

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

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseST, parkPose))
                .setLinearHeadingInterpolation(scorePoseST.getHeading(), parkPose.getHeading())
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
                        stopEverything();
                        follower.followPath(park,true);
                        setPathState(-1);
                    }
                } else actionTimer.resetTimer();
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

    public void stopEverything() {
        stopShoot();
        stopCollect();
        block();
    }
}
