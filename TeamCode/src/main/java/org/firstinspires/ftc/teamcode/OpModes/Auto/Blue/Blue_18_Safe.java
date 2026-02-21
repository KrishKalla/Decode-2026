package org.firstinspires.ftc.teamcode.OpModes.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
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
@Autonomous(name = "Blue 18 Safe Auto")
public class Blue_18_Safe extends OpMode {

    public double TURRET_ANGLE;

    public static double GateX = 17, GateY = 62, GateHeading = 156;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.7;
    private double gateIntakeTime = 1.5;

    private boolean shotWaitStarted = false;

    private LLHandler llhandler;
    private static final int alliance = 1;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions (X flipped: x = 144 - redX, heading + 180)
    private final Pose startPose           = new Pose(24.60,  126.79, Math.toRadians(180));
    private final Pose scorePose           = new Pose(55,     78,     Math.toRadians(180));
    private final Pose pickup1Pose         = new Pose(29,     59,     Math.toRadians(180));
    private final Pose midPickup1          = new Pose(54,     59);

    private final Pose gatePose            = new Pose(GateX,  GateY,  Math.toRadians(GateHeading));

    private final Pose midcenterPickupPose = new Pose(52.6,   89.2);
    private final Pose centerPickupPose    = new Pose(29,     84,     Math.toRadians(180));

    private final Pose midFarPickup        = new Pose(57.729, 31.767);
    private final Pose farPickupPose       = new Pose(29,     36,     Math.toRadians(180));
    private final Pose parkPose            = new Pose(60.872, 103,    Math.toRadians(110));

    // ---- PATH OBJECTS ----
    private PathChain Path1;
    private PathChain Path2;
    private PathChain Path3;
    private PathChain Path4;
    private PathChain Path5;
    private PathChain Path6;
    private PathChain Path7;
    private PathChain Path8;
    private PathChain Path9;
    private PathChain Path10;
    private PathChain Path11;

    private intake intake;
    private shooter shooter;
    private Turret turret;

    private final Pose goalPose = new Pose(storage.BLUE_X, storage.BLUE_Y);

    @Override
    public void init() {
        shooter = new shooter();
        intake = new intake();
        turret = new Turret();
        llhandler = new LLHandler(hardwareMap, alliance);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);
        turret.zeroTurret();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        llhandler.alliance(alliance);
        llhandler.start();

        constants.shooter.TARGET_RPM = 780;
        constants.shooter.Hood_pos = 0.69;
        TURRET_ANGLE = 132;
        buildPaths();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        llhandler.poll();
        follower.update();
        autonomousPathUpdate();

        shooter.update();
        shooter.calculateParams();
        turret.hardwareUpdate(turret.update(goalPose));

        storage.lastBlueAutoPose = follower.getPose();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Turret Angle", TURRET_ANGLE);
        telemetry.addData("Loop Time", loopTimer.seconds());
        telemetry.update();
    }

    // ---- BUILD PATHS ----
    private void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(startPose, scorePose)
        ).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(scorePose, midPickup1, pickup1Pose)
        ).setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.8).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(pickup1Pose, scorePose)
        ).setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), 0.8).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, gatePose)
        ).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading(), 0.8).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, scorePose)
        ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading(), 0.8).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierCurve(scorePose, midcenterPickupPose, centerPickupPose)
        ).setLinearHeadingInterpolation(scorePose.getHeading(), centerPickupPose.getHeading(), 0.8).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(centerPickupPose, scorePose)
        ).setLinearHeadingInterpolation(centerPickupPose.getHeading(), scorePose.getHeading(), 0.8).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, gatePose)
        ).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading(), 0.8).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, scorePose)
        ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading(), 0.8).build();

        Path10 = follower.pathBuilder().addPath(
                new BezierCurve(scorePose, midFarPickup, farPickupPose)
        ).setLinearHeadingInterpolation(scorePose.getHeading(), farPickupPose.getHeading(), 0.8).build();

        Path11 = follower.pathBuilder().addPath(
                new BezierLine(farPickupPose, parkPose)
        ).setLinearHeadingInterpolation(parkPose.getHeading(), parkPose.getHeading(), 0.8).build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, false);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        shooter.setStopper(true);
                        shotWaitStarted = false;
                        follower.followPath(Path2, false);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path3, false);
                    setPathState(3);
                }
                break;

            case 3:
                // Score and go to gate (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        follower.followPath(Path4, false);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        shooter.setStopper(true);
                        shotWaitStarted = false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // At gate - intake (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path5, false);
                        follower.setMaxPower(1);
                        shotWaitStarted = false;
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // Score and go to CENTER PICKUP
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        follower.followPath(Path6, false);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        shotWaitStarted = false;
                        setPathState(6);
                    }
                }
                break;

            case 6:
                // Center pickup back to score
                if (!follower.isBusy()) {
                    follower.followPath(Path7, false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    setPathState(7);
                }
                break;

            case 7:
                // Score and go to gate (SECOND TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path8, false);
                        shotWaitStarted = false;
                        setPathState(8);
                    }
                }
                break;

            case 8:
                // At gate - intake (SECOND TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.setMaxPower(1);
                        follower.followPath(Path9, false);
                        shotWaitStarted = false;
                        setPathState(9);
                    }
                }
                break;

            case 9:
                // Score and go to far pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        TURRET_ANGLE = 102;  // -78 + 180
                        constants.shooter.TARGET_RPM = 710;
                        constants.shooter.Hood_pos = 0.6;
                        follower.followPath(Path10, false);
                        shotWaitStarted = false;
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path11, true);
                    setPathState(11);
                }
                break;

            case 11:
                // At park - final shot
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        follower.setMaxPower(1);
                        shotWaitStarted = false;
                        setPathState(12);
                    }
                }
                break;

            case 12:
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