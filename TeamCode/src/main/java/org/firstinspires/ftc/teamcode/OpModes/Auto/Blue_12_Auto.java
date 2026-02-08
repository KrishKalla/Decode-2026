package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.experimental.Turret;
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
@Autonomous(name = "Blue 12 Auto")
public class Blue_12_Auto extends OpMode {

    private static double TURRET_ANGLE = 107;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean shotWaitStarted = false;

    private LLHandler llHandler;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    private final Pose startPose = new Pose(23.6,127.213 ,Math.toRadians(145));
    private final Pose scorePose2 = new Pose(54.000, 85.000, Math.toRadians(210));
    private final Pose pickup1Pose = new Pose(26.500, 84.000, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(28.00, 60.000, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(26.500, 36.000, Math.toRadians(180));
    private final Pose midPickup2 = new Pose(79.000, 57.000, Math.toRadians(180));
    private final Pose midPickup3 = new Pose(75.000, 30.000, Math.toRadians(180));
    private final Pose Gatepose = new Pose(20.75, 65.5, Math.toRadians(180));
    private final Pose gateToShot = new Pose(43.5,63);
    private final Pose parkpose = new Pose(56.199, 107.879, Math.toRadians(210));

    // ---- PATH OBJECTS ----
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, OpenGate, scorePickup2;
    private PathChain grabPickup3, scorePickup3;
    private PathChain Park;

    private intake intake;
    private shooter shooter;
    private Turret turret;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- Subsystems ----
        shooter = new shooter();
        intake = new intake();
        turret = new Turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llHandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, llHandler, follower.poseTracker);

        constants.shooter.TARGET_RPM = 800;
        constants.shooter.Hood_pos = 0.79;

        buildPaths();
    }

    @Override
    public void loop() {
        addTelemetry("X: ", follower.getPose().getX());
        addTelemetry("Y: ", follower.getPose().getY());
        addTelemetry("Heading: ", follower.getPose().getHeading());
        follower.update();
        autonomousPathUpdate();
        telemetry.update();
        shooter.update();
        shooter.setHood(constants.shooter.Hood_pos);
        poseStorage.lastBlueAutoPose = follower.getPose();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose2));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose2.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup1Pose.getHeading())
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose2,
                        midPickup2,
                        pickup2Pose
                ))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .build();


        OpenGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, Gatepose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), Gatepose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(Gatepose, gateToShot, scorePose2))
                .setLinearHeadingInterpolation(Gatepose.getHeading(), scorePose2.getHeading())
                .build();


        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose2,
                        midPickup3,
                        pickup3Pose
                ))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();


        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2,parkpose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), parkpose.getHeading())
                .build();



    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //Init
                follower.followPath(scorePreload);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.ON);
                turret.setTurretAngle(TURRET_ANGLE);
                shooter.setStopper(false);

                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {

                    // Shooting
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(grabPickup2, true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(99);
                    }
                }
                break;
            case 99:
                if (!follower.isBusy()){

                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.OFF);
                        shooter.setStopper(false);
                        follower.followPath(OpenGate,true);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }
                    if (shootTimer.seconds() >= 1.0) {
                        setPathState(2);
                        shotWaitStarted=false;
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //Ready to Shoot
                    follower.followPath(scorePickup2, true);
                    TURRET_ANGLE=107;
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // Shooting
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(grabPickup1, true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);

                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Shooting
                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 1.0) {

                        shooter.setStopper(true);
                        intake.setIntake(constants.INTAKE_PRESETS.ON);

                        follower.followPath(grabPickup3, true);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);

                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {

                    if (!shotWaitStarted) {
                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 2.0) {
                        shotWaitStarted = false;   // reset for next time
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.followPath(Park);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.flywheelPreset(constants.FLYWHEEL.OFF);
                    turret.zeroTurret();
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
