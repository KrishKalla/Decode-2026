package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.util.poseStorage;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Red 21 Auto")
public class Red_21 extends OpMode {

    private static double TURRET_ANGLE = -121;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean shotWaitStarted = false;

    private LLHandler llHandler;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(120.179, 127.973, Math.toRadians(36));
    private final Pose scorePose = new Pose(87.767, 82.764, Math.toRadians(-20));
    private final Pose pickup1Pose = new Pose(114, 60, Math.toRadians(0));
    private final Pose midPickup1 = new Pose(87.440, 56.941);

    private final Pose gateApproachPose = new Pose(125, 60.25, Math.toRadians(30));
    private final Pose midGatePose = new Pose(101.751, 56.946);
    private final Pose gatePose = new Pose(125, 55.7, Math.toRadians(47.5));

    private final Pose centerPickupPose = new Pose(114, 84, Math.toRadians(0));
    private final Pose midFarPickup = new Pose(86.271, 31.767);
    private final Pose farPickupPose = new Pose(114, 36, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.128, 105.965, Math.toRadians(-75));

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

    @Override
    public void init() {

        // ---- Subsystems ----
        shooter = new shooter();
        intake = new intake();
        turret = new Turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llHandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap,follower);

        constants.shooter.TARGET_RPM = 795;
        constants.shooter.Hood_pos = 0.77;

        buildPaths();
    }

    @Override
    public void start() {
        turret.setManualAngle(TURRET_ANGLE);
    }

    @Override
    public void loop() {
        addTelemetry("X: ", follower.getPose().getX());
        addTelemetry("Y: ", follower.getPose().getY());
        addTelemetry("Heading: ", follower.getPose().getHeading());
        follower.update();
        autonomousPathUpdate();
        shooter.setHood(constants.shooter.Hood_pos);
        shooter.update();
        turret.setManualAngle(TURRET_ANGLE);
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

        // Path7: Score to gate approach (second time)
        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midGatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path8: Gate approach to gate (second time)
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), gatePose.getHeading())
                .build();

        // Path9: Gate back to score (second time)
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path10: Score to gate approach (third time)
        Path10 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                midGatePose,
                                gateApproachPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        // Path11: Gate approach to gate (third time)
        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gateApproachPose,
                                gatePose
                        )
                ).setLinearHeadingInterpolation(gateApproachPose.getHeading(), gatePose.getHeading())
                .build();

        // Path12: Gate back to score (third time)
        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                gatePose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        // Path13: Score to center pickup
        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                centerPickupPose
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), centerPickupPose.getHeading())
                .build();

        // Path14: Center pickup back to score
        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                centerPickupPose,
                                scorePose
                        )
                ).setLinearHeadingInterpolation(centerPickupPose.getHeading(), scorePose.getHeading())
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
                // Init - Start to first score position
                follower.followPath(Path1);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                shooter.setStopper(false);
                setPathState(1);
                break;

            case 1:
                // Score preload and go to first pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(Path2,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(2);
                    }
                }
                break;

            case 2:
                // Return from first pickup to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Score and go to gate approach
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.GATE);

                        follower.followPath(Path4,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // Gate approach to gate
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Gate back to score
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);

                        follower.followPath(Path6,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(6);
                    }
                }
                break;

            case 6:
                // Score and go to gate approach (second time)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.GATE);

                        follower.followPath(Path7,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(7);
                    }
                }
                break;

            case 7:
                // Gate approach to gate (second time)
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(Path8, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Gate back to score (second time)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);

                        follower.followPath(Path9,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(9);
                    }
                }
                break;

            case 9:
                // Score and go to gate approach (third time)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.GATE);

                        follower.followPath(Path10,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(10);
                    }
                }
                break;

            case 10:
                // Gate approach to gate (third time)
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    follower.followPath(Path11, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Gate back to score (third time)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);

                        follower.followPath(Path12,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(12);
                    }
                }
                break;

            case 12:
                // Score and go to center pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(Path13,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(13);
                    }
                }
                break;

            case 13:
                // Center pickup back to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path14, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Score and go to far pickup
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(Path15,true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(15);
                    }
                }
                break;

            case 15:
                // Far pickup to park
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    TURRET_ANGLE=-100;
                    follower.followPath(Path16, true);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()){
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 0.5) {
                        shotWaitStarted = false;   // reset for next time
                        setPathState(17);
                    }
                }

            case 17:
                // Done - turn off subsystems
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.flywheelPreset(constants.FLYWHEEL.OFF);
                    //turret.zeroTurret();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        addTelemetry("Path State", pathState);
    }

    public void addTelemetry(String info, Object value) {
        telemetry.addData(info, value);
    }
}