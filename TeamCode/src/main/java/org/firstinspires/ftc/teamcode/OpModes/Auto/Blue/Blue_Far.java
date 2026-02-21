package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

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
@Autonomous(name = "Red Far Auto")
public class Blue_Far extends OpMode {

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private double shootingtime = 0.7;

    private boolean shotWaitStarted = false;

    private LLHandler llhandler;
    private static final int alliance = 1;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Pose definitions
    private final Pose startPose = new Pose(89.129, 7.000, Math.toRadians(90));
    private final Pose scorePose = new Pose(107, 7.000, Math.toRadians(90));

    // ---- PATH OBJECTS ----
    private PathChain Path1;

    private intake intake;
    private shooter shooter;
    private Turret turret;

    private Pose goalPose = new Pose(storage.BLUE_X, storage.BLUE_Y);

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
        //constants.shooter.TARGET_RPM = 940;
        //constants.shooter.Hood_pos = 0.6767;

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
        //shooter.update();
        turret.hardwareUpdate(turret.update(0));

        storage.lastBlueAutoPose = follower.getPose();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    private void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.871, 8.000),
                                new Pose(37.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Drive to score position
                follower.followPath(Path1, false);
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
                shooter.flywheelPreset(constants.FLYWHEEL.OFF);
                shooter.setStopper(false);
                setPathState(1);
                break;

            case 1:
                // Done - turn off subsystems
                if (!follower.isBusy()) {
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