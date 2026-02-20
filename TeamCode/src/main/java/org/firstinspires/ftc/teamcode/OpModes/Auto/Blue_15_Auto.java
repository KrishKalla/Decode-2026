//package org.firstinspires.ftc.teamcode.OpModes.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.geometry.BezierLine;
//
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.experimental.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.intake;
//import org.firstinspires.ftc.teamcode.subsystems.shooter;
//import org.firstinspires.ftc.teamcode.util.LLHandler;
//import org.firstinspires.ftc.teamcode.util.constants;
//
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Config
//@Autonomous(name = "Blue 15 Auto")
//public class Blue_15_Auto extends OpMode {
//
//    private static double TURRET_ANGLE = 115;
//    private ElapsedTime shootTimer = new ElapsedTime();
//
//    private boolean shotWaitStarted = false;
//
//    private LLHandler llHandler;
//
//    // ---- Pathing ----
//    private Follower follower;
//
//    // ---- State System ----
//    private int pathState = 0;
//
//    private final Pose startPose = new Pose(24.76, 129.48, Math.toRadians(145));
//    private final Pose scorePose2 = new Pose(54.000, 85.000, Math.toRadians(210));
//    private final Pose pickup1Pose = new Pose(25.000, 84.000, Math.toRadians(180));
//    private final Pose pickup2Pose = new Pose(25.000, 60.000, Math.toRadians(180));
//    private final Pose pickup3Pose = new Pose(25.000, 36.000, Math.toRadians(180));
//    private final Pose midPickup2 = new Pose(79.000, 57.000, Math.toRadians(180));
//    private final Pose midPickup3 = new Pose(75.000, 30.000, Math.toRadians(180));
//    private final Pose Gatepose = new Pose(19.500, 63.00, Math.toRadians(180));
//    private final Pose IntakeGatepose = new Pose(19.3, 56.2000, Math.toRadians(140));
//    private final Pose midGate = new Pose(24.7000, 55.3000, Math.toRadians(180));
//
//    // ---- PATH OBJECTS ----
//    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1;
//    private PathChain grabPickup2, scorePickup2;
//    private PathChain grabPickup3, scorePickup3;
//    private PathChain grabGate, intakeGate;
//    private Path scoreGate;
//
//    private intake intake;
//    private shooter shooter;
//    private Turret turret;
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // ---- Subsystems ----
//        shooter = new shooter();
//        intake = new intake();
//        turret = new Turret();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setPose(startPose);
//
//        shooter.init(hardwareMap, llHandler);
//        intake.init(hardwareMap);
//        turret.init(hardwareMap, llHandler, follower.poseTracker);
//
//        constants.shooter.TARGET_RPM = 810;
//        constants.shooter.Hood_pos = 0.81;
//
//        buildPaths();
//    }
//
//    @Override
//    public void loop() {
//        addTelemetry("X: ", follower.getPose().getX());
//        addTelemetry("Y: ", follower.getPose().getY());
//        addTelemetry("Heading: ", follower.getPose().getHeading());
//        follower.update();
//        autonomousPathUpdate();
//        telemetry.update();
//        shooter.update();
//    }
//
//
//    // ---- BUILD PATHS ----
//    private void buildPaths() {
//
//        scorePreload = new Path(new BezierLine(startPose, scorePose2));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose2.getHeading());
//
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose2, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, scorePose2))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
//                .build();
//
//
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        scorePose2,
//                        midPickup2,
//                        pickup2Pose
//                ))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, scorePose2))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
//                .build();
//
//
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        scorePose2,
//                        midPickup3,
//                        pickup3Pose
//                ))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, scorePose2))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
//                .build();
//
//        grabGate = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose2, Gatepose))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), Gatepose.getHeading())
//                .build();
//
//        intakeGate = follower.pathBuilder()
//                .addPath(new BezierCurve(Gatepose, midGate, IntakeGatepose))
//                .setLinearHeadingInterpolation(Gatepose.getHeading(), IntakeGatepose.getHeading())
//                .build();
//
//        scoreGate = new Path(new BezierLine(Gatepose, scorePose2));
//        scoreGate.setLinearHeadingInterpolation(Gatepose.getHeading(), scorePose2.getHeading());
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                //Init
//                follower.followPath(scorePreload);
//                intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                shooter.flywheelPreset(constants.FLYWHEEL.ON);
//                turret.setTurretAngle(TURRET_ANGLE);
//                shooter.setStopper(false);
//
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//
//                    // Shooting
//                    if (!shotWaitStarted) {
//                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 1.0) {
//
//                        shooter.setStopper(true);
//                        intake.setIntake(constants.INTAKE_PRESETS.ON);
//
//                        follower.followPath(grabPickup2, true);
//                        shotWaitStarted = false;   // reset for next time
//                        setPathState(2);
//                    }
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    //Ready to Shoot
//                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                    shooter.setStopper(false);
//
//                    follower.followPath(scorePickup2, true);
//                    setPathState(99);
//                }
//                break;
//            case 99:
//                if (!follower.isBusy()) {
//                    // Shooting
//                    if (!shotWaitStarted) {
//                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 1.0) {
//
//                        shooter.setStopper(true);
//                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
//
//                        follower.followPath(grabGate, true);
//                        shotWaitStarted = false;   // reset for next time
//                        setPathState(100);
//                    }
//                }
//                break;
//            case 100:
//                if (!follower.isBusy()) {
//                    //Ready to Shoot
//                    follower.followPath(intakeGate, true);
//                    if (!shotWaitStarted)
//                    {
//                        shootTimer.reset();
//                        shotWaitStarted=true;
//                    }
//                    if (shootTimer.seconds() >= 0.25)
//                    {
//                        intake.setIntake(constants.INTAKE_PRESETS.ON);
//                        shotWaitStarted = false;
//                        setPathState(101);
//                    }
//
//                }
//                break;
//
//            case 101:
//                if (!follower.isBusy()) {
//
//                    if (!shotWaitStarted) {
//
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 1.0){
//                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                        shooter.setStopper(false);
//                        follower.followPath(scoreGate, true);
//                        shotWaitStarted = false;
//                        setPathState(3);
//                    }
//
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    // Shooting
//                    if (!shotWaitStarted) {
//                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 1.0) {
//
//                        shooter.setStopper(true);
//                        intake.setIntake(constants.INTAKE_PRESETS.ON);
//
//                        follower.followPath(grabPickup1, true);
//                        shotWaitStarted = false;   // reset for next time
//                        setPathState(4);
//                    }
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                    shooter.setStopper(false);
//
//                    follower.followPath(scorePickup1, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    // Shooting
//                    if (!shotWaitStarted) {
//                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 1.0) {
//
//                        shooter.setStopper(true);
//                        intake.setIntake(constants.INTAKE_PRESETS.ON);
//
//                        follower.followPath(grabPickup3, true);
//                        shotWaitStarted = false;   // reset for next time
//                        setPathState(6);
//                    }
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                    shooter.setStopper(false);
//
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//
//                    if (!shotWaitStarted) {
//                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//                        shootTimer.reset();
//                        shotWaitStarted = true;
//                    }
//
//                    if (shootTimer.seconds() >= 2.0) {
//                        shotWaitStarted = false;   // reset for next time
//                        setPathState(8);
//                    }
//                }
//                break;
//            case 8:
//                if (!follower.isBusy()){
//                intake.setIntake(constants.INTAKE_PRESETS.OFF);
//                shooter.flywheelPreset(constants.FLYWHEEL.OFF);
//                setPathState(-1); // Done
//            }
//            break;
//        }
//    }
//
//    public void setPathState(int pState) { pathState = pState; }
//
//
//    public void addTelemetry(String info, Object value) {
//        telemetry.addData(info, value);
//    }
//
//}
