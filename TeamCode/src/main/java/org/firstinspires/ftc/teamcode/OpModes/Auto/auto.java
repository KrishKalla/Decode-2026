package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Test Close Auto")
public class Auto extends OpMode {

    private LLHandler llHandler;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    private final Pose startPose = new Pose(26.1, 128.6, Math.toRadians(145));
    private final Pose scorePose = new Pose(55.000, 84.000, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(23.000, 84.000, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(23.000, 60.000, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(23.000, 36.000, Math.toRadians(180));
    private final Pose midPickup2 = new Pose(79.000, 57.000, Math.toRadians(180));
    private final Pose midPickup3 = new Pose(74.000, 30.000, Math.toRadians(180));

    // ---- PATH OBJECTS ----
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- Subsystems ----
        shooter shooter = new shooter();
        intake intake = new intake();
        turret turret = new turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        constants.shooter.TARGET_RPM = 800;
        constants.shooter.Hood_pos = 0.75;

        shooter.init(hardwareMap, llHandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, llHandler);

        addTelemetry("position", follower.getPose());
        
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.update();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        midPickup2,
                        pickup2Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        midPickup3,
                        pickup3Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Score Preload
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // Done
                }
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; }


    public void addTelemetry(String info, Object value) {
        telemetry.addData(info, value);
    }

}
