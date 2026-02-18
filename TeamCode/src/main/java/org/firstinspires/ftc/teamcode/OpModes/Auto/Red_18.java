package org.firstinspires.ftc.teamcode.OpModes.Auto;

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
@Autonomous(name = "Red 18 Auto")
public class Red_18 extends OpMode {

    public double TURRET_ANGLE;

    public static double GateX=129, GateY=62, GateHeading=30;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.7;
    private double gateIntakeTime = 2;

    private boolean shotWaitStarted = false;

    private LLHandler llhandler;
    private static final int alliance = 0;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(119.40, 126.79, Math.toRadians(0));
    private final Pose scorePose = new Pose(89, 84, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(115, 60, Math.toRadians(0));
    private final Pose midPickup1 = new Pose(87.440, 56.941);

    private final Pose gatePose = new Pose(GateX, GateY, Math.toRadians(GateHeading));

    private final Pose centerPickupPose = new Pose(115, 82, Math.toRadians(0));
    private final Pose midFarPickup = new Pose(86.271, 31.767);
    private final Pose farPickupPose = new Pose(115, 36, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.128, 100, Math.toRadians(-70));

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
        turret.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        llhandler.alliance(alliance);
        llhandler.start();

        constants.shooter.TARGET_RPM = 760;
        constants.shooter.Hood_pos = 0.71;
        TURRET_ANGLE = -135;
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


        shooter.setHood(constants.shooter.Hood_pos);
        shooter.update();
        turret.update(TURRET_ANGLE);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Loop Time", loopTimer.seconds());
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

        // Path4: Score directly to gate (FIRST TIME)
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        // Path5: Gate back to score (FIRST TIME)
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path6: Score to center pickup
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                centerPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), centerPickupPose.getHeading())
                .build();

        // Path7: Center pickup back to score
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                centerPickupPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(centerPickupPose.getHeading(), scorePose.getHeading())
                .build();

        // Path8: Score directly to gate (SECOND TIME)
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        // Path9: Gate back to score (SECOND TIME)
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path10: Score to far pickup and park
        Path10 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midFarPickup,
                                farPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), farPickupPose.getHeading())
                .build();
        Path11 = follower.pathBuilder().addPath(
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
                // Init - Start to first score position
                follower.followPath(Path1,false);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                setPathState(1);
                break;
            case 1:
                // Go to first pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted=true;
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    }
                    if (shootTimer.seconds()>= shootingtime){
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        shooter.setStopper(true);

                        shotWaitStarted = false;
                        follower.followPath(Path2,false);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                // Return from first pickup to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path3,false);
                    setPathState(3);
                }
                break;

            case 3:
                // Score and go directly to gate (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= shootingtime) {
                        follower.followPath(Path4,false);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        shooter.setStopper(true);
                        shotWaitStarted = false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // At gate - intake for 1.5 seconds (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= gateIntakeTime) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path5,false);
                        shotWaitStarted = false;
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // Score and go to CENTER PICKUP
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= shootingtime) {
                        follower.followPath(Path6,false);
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
                    follower.followPath(Path7,false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    GateX=129;
                    GateY=61;
                    GateHeading=30;
                    setPathState(7);
                }
                break;

            case 7:
                // Score and go directly to gate (SECOND TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path8,false);
                        shotWaitStarted = false;
                        setPathState(8);
                    }
                }
                break;

            case 8:
                // At gate - intake for 1.5 seconds (SECOND TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= gateIntakeTime) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path9,false);
                        shotWaitStarted = false;
                        setPathState(9);
                    }
                }
                break;

            case 9:
                // Score and go to far pickup, then park
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        TURRET_ANGLE=-78;
                        constants.shooter.TARGET_RPM = 700;
                        constants.shooter.Hood_pos = 0.60;
                        follower.followPath(Path10,false);
                        shotWaitStarted = false;
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()){
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path11, true);
                    setPathState(11);
                }
                break;

            case 11:
                // At park - final shot
                if(!follower.isBusy()){
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= shootingtime) {
                        shotWaitStarted = false;
                        setPathState(12);
                    }
                }
                break;
            case 12:
                // Done - turn off subsystems
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