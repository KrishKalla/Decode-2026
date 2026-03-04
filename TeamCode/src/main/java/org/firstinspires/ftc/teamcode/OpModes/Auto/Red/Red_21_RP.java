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
@Autonomous(name = "Red 21 Auto - RP")
public class Red_21_RP extends OpMode {

    public static double TURRET_ANGLE = -132;
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.45;
    private static double gateIntakeTime = 2;
    private boolean IsShot=false;

    private boolean Auto_hood = true;
    private boolean shotWaitStarted = false;
    private boolean moveshootfinished = false;

    private LLHandler llhandler;
    private static final int alliance = 0;

    //Moving While Shooting
    public static double Power=0.5;
    public static double ShootingMoment=1.4;
    public static double ShootingHood=0.3;
    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(116.40, 126.79, Math.toRadians(0));
    private final Pose scorePose = new Pose(92, 76, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(113, 60, Math.toRadians(0));
    private final Pose midPickup1 = new Pose(70, 55.5);


    private final Pose gateApproachPose = new Pose(129, 60, Math.toRadians(20));

    private final Pose midcenterPickupPose = new Pose(90,84.5);
    private final Pose centerPickupPose = new Pose(113, 84, Math.toRadians(0));

    private final Pose midFarPickup = new Pose(86.271, 31.767);
    private final Pose farPickupPose = new Pose(113, 36, Math.toRadians(0));
    private final Pose parkPose = new Pose(83.128, 103, Math.toRadians(-45));

    // ---- PATH OBJECTS ----
    private PathChain Path1;
    private PathChain Path3;
    private PathChain Path4;
    private PathChain Path6;
    private PathChain Path7;
    private PathChain Path8;
    private PathChain Path12;
    private PathChain Path13;

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
        constants.shooter.TARGET_RPM = 400;//790 Original
        constants.shooter.Target_Hood = ShootingHood;
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
        intake.update();
        //turret.hardwareUpdate(turret.update(goalPose));

        storage.lastRedAutoPose = follower.getPose();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Target", turret.getTargetAngle());
        telemetry.addData("Turret Current", turret.getCurrentAngle());
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("lldist", llhandler.getLatestResult()[2]);
        telemetry.update();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {
        // Path1: Start to score position
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                midPickup1,
                                pickup1Pose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
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
                        new BezierLine(
                                scorePose,
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
                                parkPose
                        )
                ).setLinearHeadingInterpolation(parkPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(Power);
                //Shoot while moving
                if (!shotWaitStarted) {
                    follower.followPath(Path1, false);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.flywheelPreset(constants.FLYWHEEL.ON);
                    shooter.setStopper(false);
                    shootTimer.reset();
                    shotWaitStarted = true;
                }
                if (shootTimer.seconds() >= ShootingMoment && !IsShot) {
                    intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);

                }
                if (shootTimer.seconds() >= ShootingMoment+0.4){
                    follower.setMaxPower(1);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(true);
                    IsShot=true;
                    shotWaitStarted = false;
                    setPathState(1);
                }
                break;

            case 1:
                // go to near pickup
                //constants.shooter.Target_Hood=0.69;
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                setPathState(2);
                break;

            case 2:
                // Return from gate to score
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
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path4, false);
                        shotWaitStarted = false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                // Wait at gate, then return to score (FIRST TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        follower.setMaxPower(0.2);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= gateIntakeTime-0.5) {
                        follower.setMaxPower(1);
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(Path6, false);
                        shotWaitStarted = false;
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // Score and go to gate (Second TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path4, false);
                        shotWaitStarted = false;
                        setPathState(6);
                    }
                }
                break;

            case 6:
                // Wait at gate, then return to score (Second TIME)
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
                        setPathState(7);
                    }
                }
                break;

            case 7:
                // Score and go to gate (Third TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);
                        follower.followPath(Path4, false);
                        shotWaitStarted = false;
                        setPathState(8);
                    }
                }
                break;

            case 8:
                // Wait at gate, then return to score (Third TIME)
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
                        setPathState(9);
                    }
                }
                break;

            case 9:
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
                        setPathState(10);
                    }
                }
                break;

            case 10:
                // Center pickup back to score
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(Path8, false);
                    setPathState(11);
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
                // Far pickup back to Park
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    //constants.shooter.TARGET_RPM = 720;
                    //constants.shooter.Hood_pos = 0.59;
                    shooter.setStopper(false);
                    follower.followPath(Path13, false);
                    setPathState(13);
                }
                break;

            case 13:
                // Score and then Gate approach to gate (THIRD TIME)
                if (!follower.isBusy()) {
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= shootingtime) {
                        shooter.setStopper(true);
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