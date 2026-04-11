package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Red Far Auto")
public class Red_Far extends OpMode {

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();

    private double shootingtime = 0.45;
    private boolean shotWaitStarted = false;

    private LLHandler llhandler;

    private Follower follower;
    private int pathState = 0;

    // -------------------------------------------------------
    // POSES
    // -------------------------------------------------------
    private final Pose startPose          = new Pose(114.4, 20, Math.toRadians(0));
    private final Pose tinyMovePose       = new Pose(105, 20, Math.toRadians(0));

    private final Pose scorePose          = new Pose(86, 20, Math.toRadians(0));

    // Third spike mark
    private final Pose midSpike3          = new Pose(100, 35);
    private final Pose spike3Pose         = new Pose(120, 40, Math.toRadians(0));

    // Far field intake positions (no gate)
    private final Pose midFarIntake1      = new Pose(106, 14);
    private final Pose farIntakePose1     = new Pose(131, 11, Math.toRadians(0));

    private final Pose midFarIntake2      = new Pose(106, 18);
    private final Pose farIntakePose2     = new Pose(131, 16, Math.toRadians(0));

    // ---- PATH OBJECTS ----
    private PathChain PathTinyMove;
    private PathChain PathToSpike3;
    private PathChain PathSpike3Back;
    private PathChain PathToFarIntake1;
    private PathChain PathFarIntake1Back;
    private PathChain PathToFarIntake2;
    private PathChain PathFarIntake2Back;

    private intake intake;
    private shooter shooter;
    private turret turret;

    private Pose goalPose = new Pose(storage.RED_X, storage.RED_Y);

    // -------------------------------------------------------
    @Override
    public void init() {
        shooter = new shooter();
        intake  = new intake();
        turret  = new turret();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        buildPaths();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        autonomousPathUpdate();
        shooter.update();
        shooter.calculateParams();
        intake.update();
        turret.update(goalPose);
        turret.periodic();

        storage.lastRedAutoPose = follower.getPose();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Target", turret.getTargetAngle());
        telemetry.addData("Turret Current", turret.getCurrentAngle());
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.update();
    }

    // -------------------------------------------------------
    private void buildPaths() {

        PathTinyMove = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tinyMovePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), tinyMovePose.getHeading())
                .build();

        PathToSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midSpike3, spike3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spike3Pose.getHeading())
                .build();

        PathSpike3Back = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pose, scorePose))
                .setLinearHeadingInterpolation(spike3Pose.getHeading(), scorePose.getHeading())
                .build();

        PathToFarIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midFarIntake1, farIntakePose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), farIntakePose1.getHeading())
                .build();

        PathFarIntake1Back = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose1, scorePose))
                .setLinearHeadingInterpolation(farIntakePose1.getHeading(), scorePose.getHeading())
                .build();

        PathToFarIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midFarIntake2, farIntakePose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), farIntakePose2.getHeading())
                .build();

        PathFarIntake2Back = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose2, scorePose))
                .setLinearHeadingInterpolation(farIntakePose2.getHeading(), scorePose.getHeading())
                .build();
    }

    // -------------------------------------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── STATE 0 : spin up + tiny move ───────────────────────────
            case 0:
                follower.setMaxPower(0.4);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                follower.followPath(PathTinyMove, false);
                setPathState(1);
                break;

            // ── STATE 1 : wait, then shoot preload ──────────────────────
            case 1:
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        follower.setMaxPower(1);
                        shotWaitStarted = false;
                        setPathState(2);
                    }
                }
                break;

            // ── STATE 2 : drive to spike 3 with intake on ───────────────
            case 2:
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(PathToSpike3, false);
                    setPathState(3);
                }
                break;

            // ── STATE 3 : return from spike 3, intake still on ──────────
            case 3:
                if (!follower.isBusy()) {
                    // intake stays ON during the return trip to collect balls
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(PathSpike3Back, false);
                    setPathState(4);
                }
                break;

            // ── STATE 4 : shoot spike3 balls, then head to far intake 1 ─
            case 4:
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(PathToFarIntake1, false);
                        shotWaitStarted = false;
                        setPathState(5);
                    }
                }
                break;

            // ── STATE 5 : at far intake 1, just drive back (no wait) ────
            case 5:
                if (!follower.isBusy()) {
                    // intake stays ON — just turn around and come back
                    shooter.setStopper(false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    follower.followPath(PathFarIntake1Back, false);
                    setPathState(6);
                }
                break;

            // ── STATE 6 : shoot far intake 1 balls, go to far intake 2 ──
            case 6:
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(PathToFarIntake2, false);
                        shotWaitStarted = false;
                        setPathState(7);
                    }
                }
                break;

            // ── STATE 7 : at far intake 2, just drive back (no wait) ────
            case 7:
                if (!follower.isBusy()) {
                    // intake stays ON — just turn around and come back
                    shooter.setStopper(false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    follower.followPath(PathFarIntake2Back, false);
                    setPathState(8);
                }
                break;

            // ── STATE 8 : final shot then shut down ──────────────────────
            case 8:
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        shotWaitStarted = false;
                        setPathState(9);
                    }
                }
                break;

            case 9:
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.OFF);
                setPathState(-1);
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