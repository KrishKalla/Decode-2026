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
@Autonomous(name = "Red 21 Auto - Safe")
public class Red_21_Safe extends OpMode {

    public static double turret_offset=0;
    public static double gateposex = 131;
    public static double gateposey = 58.5;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.45;
    private static double gateIntakeTime = 1.3;
    private boolean IsShot=false;

    private boolean Auto_hood = true;
    private boolean shotWaitStarted = false;
    private boolean moveshootfinished = false;

    private LLHandler llhandler;
    private static final int alliance = 0;

    //Moving While Shooting
    public static double Power=0.5;
    public static double ShootingMoment=1.67;
    public static double ShootingHood=0.45 ;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(115.23, 125.38, Math.toRadians(0));
    private final Pose scorePose = new Pose(86, 76, Math.toRadians(0));
    private final Pose FirstscorePose = new Pose(93, 85, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(120, 61, Math.toRadians(0));
    private final Pose midPickup1 = new Pose(88.2, 60);

    private final Pose gateApproachPose = new Pose(131, 58.5, Math.toRadians(20));
    private final Pose midgatePose = new Pose(106,60);

    private final Pose midcenterPickupPose = new Pose(90,84);
    private final Pose centerPickupPose = new Pose(120, 80, Math.toRadians(0));

    private final Pose midFarPickup = new Pose(86.271, 36);
    private final Pose farPickupPose = new Pose(120, 40, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.128, 103, Math.toRadians(-45));

    // ---- PATH OBJECTS ----
    private PathChain Path1;
    private PathChain Path2;
    private PathChain Path3;
    private PathChain Path4;
    private PathChain Path6;
    private PathChain Path7;
    private PathChain Path8;
    private PathChain Path9;
    private PathChain Path11;
    private PathChain Path12;
    private PathChain Path13;
    private PathChain Path14;
    private PathChain Path16;

    private intake intake;
    private shooter shooter;
    private turret turret;

    private Pose goalPose = new Pose(storage.RED_X, storage.RED_Y);

    @Override
    public void init() {

        // ---- Subsystems ----
        shooter = new shooter();
        intake = new intake();
        turret = new turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap,follower);
        //turret.zeroTurret();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        constants.shooter.TARGET_RPM = 1440;
        constants.shooter.Target_Hood = 0.64;
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
        intake.update();

        turret.update(goalPose);
        turret.periodic();

        if (follower.getPose().equals(new Pose(0, 0, 0))) {

        } else{
            storage.lastRedAutoPose = follower.getPose();
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Target", turret.getTargetAngle());
        telemetry.addData("Turret Current", turret.getCurrentAngle());
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.update();
    }

    // ---- BUILD PATHS ----
    private void buildPaths() {
        // Path1: Start to score position
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                FirstscorePose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midPickup1,
                                pickup1Pose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path3: First pickup back to score
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                pickup1Pose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Path4: Score to gate approach
        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midgatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();


        // Path6: Gate back to score
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), scorePose.getHeading())
                .build();

        // Path7: Score to center pickup (MOVED HERE - between 1st and 2nd gate)
        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midcenterPickupPose,
                                centerPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), centerPickupPose.getHeading())
                .build();

        // Path8: Center pickup back to score (MOVED HERE)
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                centerPickupPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(centerPickupPose.getHeading(), scorePose.getHeading())
                .build();

        // Path9: Score to gate approach (second time)
        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midgatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path11: Gate back to score (second time)
        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), scorePose.getHeading())
                .build();

        // Path12: Intake Third Row
        Path12 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midFarPickup,
                                farPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), farPickupPose.getHeading())
                .build();

        // Path13: score far pickup
        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                farPickupPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(farPickupPose.getHeading(), scorePose.getHeading())
                .build();

        // Path14: Go to gate
        Path14 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midgatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path16: Gate to park
        Path16 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                parkPose
                        )
                ).setLinearHeadingInterpolation(parkPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.9);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                follower.followPath(Path1, false);
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
                        constants.shooter.TARGET_RPM = 1500;
                        constants.shooter.Target_Hood = 0.69;
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
                // Return from spike mark to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path3, false);
                    setPathState(3);
                }
                break;

            case 3:
                // Score and go to gate approach (FIRST TIME)
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
                        follower.followPath(Path4, false);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // Wait at gate, then return to score (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime) {
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path6, false);
                        shotWaitStarted = false;
                        setPathState(6);
                    }
                }
                break;

            case 6:
                // Score and go to CENTER pickup
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
                        follower.followPath(Path7, false);
                        shotWaitStarted = false;
                        setPathState(7);
                    }
                }
                break;

            case 7:
                // Center pickup back to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path8, false);
                    setPathState(8);
                }
                break;

            case 8:
                // Score and go to gate approach (SECOND TIME)
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
                        follower.followPath(Path9, false);
                        shotWaitStarted = false;
                        setPathState(10);
                    }
                }
                break;

            case 10:
                // Wait at gate, then return to score (SECOND TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime) {
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path11, false);
                        shotWaitStarted = false;
                        setPathState(11);
                    }
                }
                break;

            case 11:
                // Score and go to far pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path12, false);
                        shotWaitStarted = false;
                        setPathState(12);
                    }
                }
                break;

            case 12:
                // Far pickup back to score (THIRD GATE PREP)
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path13, false);
                    setPathState(13);
                }
                break;

            case 13:
                // Score and then Gate approach to gate (THIRD TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        constants.shooter.TARGET_RPM = 1450;
                        constants.shooter.Target_Hood = 0.59;
                        follower.setMaxPower(1);
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path14, false);
                        shotWaitStarted = false;
                        setPathState(15);
                    }
                }
                break;

            case 15:
                // Wait at gate, then return to score (THIRD TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime) {
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path16, false);
                        shotWaitStarted = false;
                        setPathState(16);
                    }
                }
                break;

            case 16:
                // Far pickup to park
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path16, false);
                    setPathState(17);
                }
                break;

            case 17:
                // Final shot at park
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
                        setPathState(18);
                    }
                }
                break;

            case 18:
                // Done - turn off subsystems
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.flywheelPreset(constants.FLYWHEEL.OFF);
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