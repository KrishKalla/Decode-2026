package org.firstinspires.ftc.teamcode.OpModes.Auto.Test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous(name = "Blue 21 Auto - RP")
public class Allison_Auto extends OpMode {


    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(29.5, 126.5, Math.toRadians(0));
    private final Pose scorePose = new Pose(86, 76, Math.toRadians(0));

    // Define Your Path
    private PathChain Path1;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {
        // Build Your Path
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            case -1:
                requestOpModeStop();
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        telemetry.addData("Path State", pathState);
    }
}