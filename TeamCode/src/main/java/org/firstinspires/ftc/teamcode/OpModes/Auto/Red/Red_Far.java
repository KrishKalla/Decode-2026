package org.firstinspires.ftc.teamcode.OpModes.Auto.Blue;

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

    private double shootingtime = 0.8;
    private boolean shotWaitStarted = false;

    private LLHandler llhandler;
    private int alliance = 1;

    private Follower follower;
    private int pathState = 0;

    private static double wall_y=144-7.65;
    private static double wall_x=144-56.3;

    // -------------------------------------------------------
    // POSES
    // -------------------------------------------------------
    private final Pose startPose          = new Pose(wall_x, wall_y, Math.toRadians(0));

    private final Pose scorePose          = new Pose(wall_x, 12, Math.toRadians(0));

    // Third spike mark
    private final Pose midSpike3          = new Pose(wall_x, 40);
    private final Pose spike3Pose         = new Pose(120, 36, Math.toRadians(0));

    // Far field intake positions (no gate)
    private final Pose farIntakePose1     = new Pose(133, wall_y, Math.toRadians(0));

    private final Pose farIntakePose2     = new Pose(133, 28, Math.toRadians(0));

    private final Pose midIntakePose     = new Pose(130, 14, Math.toRadians(0));

    // ---- PATH OBJECTS ----
    private PathChain PathTinyMove;
    private PathChain PathToSpike3;
    private PathChain PathSpike3Back;
    private PathChain PathToFarIntake1;
    private PathChain PathFarIntake1Back;
    private PathChain PathToFarIntake2;
    private PathChain PathFarIntake2Back;
    private PathChain Shakeitoff1_in;
    private PathChain Shakeitoff1_out;
    private PathChain Shakeitoff2_in;
    private PathChain Shakeitoff2_out;

    private intake intake;
    private shooter shooter;
    private turret turret;

    private final double autoRedGoal = storage.RED_X;

    private Pose goalPose = new Pose(autoRedGoal-2, storage.RED_Y);

    // -------------------------------------------------------
    @Override
    public void init() {
        shooter = new shooter();
        intake  = new intake();
        turret  = new turret();

        llhandler = new LLHandler(hardwareMap, alliance);
        llhandler.alliance(alliance);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        constants.intake.TRANSFER_POWER=0.67;

        buildPaths();
    }

    @Override
    public void start() {
        llhandler.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        llhandler.poll();

        follower.update();
        autonomousPathUpdate();
        shooter.update();
        shooter.calculateParams(2000,0);
        intake.update();
        turret.update(goalPose);
        turret.periodic();

        if (follower.getPose().equals(new Pose(0, 0, 0))) {

        } else{
            storage.lastBlueAutoPose = follower.getPose();
        }

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
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierLine(scorePose, farIntakePose1))
                .setLinearHeadingInterpolation(Math.toRadians(180), farIntakePose1.getHeading())
                .build();

        PathFarIntake1Back = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose1, scorePose))
                .setLinearHeadingInterpolation(farIntakePose1.getHeading(), scorePose.getHeading())
                .build();

        PathToFarIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, farIntakePose2))
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(160))
                .build();

        PathFarIntake2Back = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose2, scorePose))
                .setLinearHeadingInterpolation(farIntakePose2.getHeading(), scorePose.getHeading())
                .build();
        Shakeitoff1_in = follower.pathBuilder()
                .addPath(new BezierLine(midIntakePose, farIntakePose1))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
        Shakeitoff1_out = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose1, midIntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
        Shakeitoff2_in = follower.pathBuilder()
                .addPath(new BezierLine(midIntakePose, farIntakePose2))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
        Shakeitoff2_out = follower.pathBuilder()
                .addPath(new BezierLine(farIntakePose2, midIntakePose))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
    }

    // -------------------------------------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── STATE 0 : spin up + tiny move ───────────────────────────
            case 0:
                follower.setMaxPower(0.6);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                if (!shotWaitStarted) {
                    follower.followPath(PathTinyMove, false);
                    shotWaitStarted = true;
                    shootTimer.reset();
                }
                if (shootTimer.seconds() >= 2) {
                    shotWaitStarted = false;
                    setPathState(1);
                }
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
                        setPathState(99);
                    }
                }
                break;

            case 99:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff1_out);
                    setPathState(100);
                }

            case 100:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff1_in);
                    setPathState(5);
                }


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
                        setPathState(101);
                    }
                }
                break;

            case 101:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff2_out);
                    setPathState(102);
                }
            case 102:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff2_in);
                    setPathState(7);
                }

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
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(PathToFarIntake1, false);
                        shotWaitStarted = false;
                        setPathState(103);
                    }
                }
                break;

            case 103:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff1_out);
                    setPathState(104);
                }
            case 104:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff1_in);
                    setPathState(9);
                }

            case 9:
                if (!follower.isBusy()) {
                    // intake stays ON — just turn around and come back
                    shooter.setStopper(false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    follower.followPath(PathFarIntake1Back, false);
                    setPathState(10);
                }
                break;
            case 10:
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
                        setPathState(105);
                    }
                }
                break;

            case 105:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff2_out);
                    setPathState(106);
                }
            case 106:
                if (!follower.isBusy()) {
                    follower.followPath(Shakeitoff2_in);
                    setPathState(11);
                }

            case 11:
                if (!follower.isBusy()) {
                    // intake stays ON — just turn around and come back
                    shooter.setStopper(false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    follower.followPath(PathFarIntake2Back, false);
                    setPathState(12);
                }
                break;
            case 12:
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
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    // intake stays ON — just turn around and come back
                    shooter.setStopper(false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    follower.followPath(PathFarIntake1Back, false);
                    setPathState(14);
                }
                break;
            case 14:
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
                        shotWaitStarted = false;
                        setPathState(15);
                    }
                }
                break;


            case 15:
                constants.intake.TRANSFER_POWER=1;
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