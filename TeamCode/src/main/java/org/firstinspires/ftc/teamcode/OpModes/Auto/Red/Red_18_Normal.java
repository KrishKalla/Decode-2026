package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

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
@Autonomous(name = "Red 18 Nomarl Auto")
public class Red_18_Normal extends OpMode {

    public static double TURRET_ANGLE = -132;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.7;
    private static double gateIntakeTime = 1.0;
    private boolean IsShot=false;

    private boolean Auto_hood = true;
    private boolean shotWaitStarted = false;
    private boolean moveshootfinished = false;

    private LLHandler llhandler;
    private static final int alliance = 0;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(118.40, 126.79, Math.toRadians(0));
    private final Pose scorePose = new Pose(89, 78, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(115, 59, Math.toRadians(0));
    private final Pose midPickup1 = new Pose(90, 59);

    private final Pose gateApproachPose = new Pose(127, 61, Math.toRadians(30));
    private final Pose midGatePose = new Pose(120, 54);
    private final Pose gatePose = new Pose(129, 55, Math.toRadians(47.5));

    private final Pose midcenterPickupPose = new Pose(91.4,89.2);
    private final Pose centerPickupPose = new Pose(115, 84, Math.toRadians(0));

    private final Pose midFarPickup = new Pose(86.271, 31.767);
    private final Pose farPickupPose = new Pose(115, 36, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.128, 103, Math.toRadians(-70));

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
    private PathChain Path12;
    private PathChain Path13;
    private PathChain Path14;
    private PathChain Path15;
    private PathChain Path16;

    private intake intake;
    private shooter shooter;
    private Turret turret;

    private Pose goalPose = new Pose(storage.RED_X, storage.RED_Y);

    @Override
    public void init() {

        // ---- Subsystems ----
        shooter = new shooter();
        intake = new intake();
        turret = new Turret();
        llhandler = new LLHandler(hardwareMap, alliance);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap,follower);
        turret.zeroTurret();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        llhandler.alliance(alliance);
        llhandler.start();
        constants.shooter.TARGET_RPM = 790;
        constants.shooter.Hood_pos = 0.69;
        storage.RED_X=138;
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
        //shooter.calculateParams();
        shooter.update();
        turret.hardwareUpdate(turret.update(goalPose));

        storage.lastRedAutoPose = follower.getPose();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("lldist", llhandler.getLatestResult()[2]);
        telemetry.update();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {
        // Path1: Start to score position
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Path2: Score to first pickup
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
                                midGatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path5: Gate approach to gate
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), gatePose.getHeading())
                .build();

        // Path6: Gate back to score
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
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
                                midGatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path10: Gate approach to gate (second time)
        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), gatePose.getHeading())
                .build();

        // Path11: Gate back to score (second time)
        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path12: Score to gate approach (third time)
        Path12 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midGatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path13: Gate approach to gate (third time)
        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), gatePose.getHeading())
                .build();

        // Path14: Gate back to score (third time)
        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path15: Score to far pickup
        Path15 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midFarPickup,
                                farPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), farPickupPose.getHeading())
                .build();

        // Path16: Far pickup to park
        Path16 = follower.pathBuilder().addPath(
                        new BezierLine(
                                farPickupPose,
                                parkPose
                        )
                ).setLinearHeadingInterpolation(parkPose.getHeading(), parkPose.getHeading())
                .build();
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
                // Wait to shoot, then go to near pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if(shootTimer.seconds()>=0.5&&!IsShot)
                    {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds() >= shootingtime+0.5) {
                        IsShot=true;
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
                // Return from near pickup to score
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
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.GATE);
                        follower.followPath(Path4, false);
                        shotWaitStarted = false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // Gate approach to gate (FIRST TIME)
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(Path5, false);
                    setPathState(5);
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
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
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
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.GATE);
                        follower.followPath(Path9, false);
                        shotWaitStarted = false;
                        setPathState(9);
                    }
                }
                break;

            case 9:
                // Gate approach to gate (SECOND TIME)
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(Path10, false);
                    setPathState(10);
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
                        follower.followPath(Path15, false);
                        constants.shooter.TARGET_RPM = 720;
                        constants.shooter.Hood_pos = 0.59;
                        shotWaitStarted = false;
                        setPathState(12);
                    }
                }
                break;

            case 12:
                // Far pickup to park
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path16, false);

                    setPathState(13);
                }
                break;

            case 13:
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
                        setPathState(14);
                    }
                }
                break;

            case 14:
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