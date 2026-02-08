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

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Red Far")
public class RED_Far extends OpMode {

    private static double TURRET_ANGLE = -2;
    private ElapsedTime shootTimer = new ElapsedTime();

    private boolean shotWaitStarted = false;

    private LLHandler llHandler;

    // ---- Pathing ----
    private Follower follower;

    // ---- State System ----
    private int pathState = 0;

    // Blue: 24.76, 129.48, 145° → Red: 119.24, 129.48, 215°
    private final Pose startPose = new Pose(109,9,Math.toRadians(90));
    // Blue: 54.0, 85.0, 210° → Red: 90.0, 85.0, 150°
    private final Pose parkpose = new Pose(119, 9, Math.toRadians(90));

    // ---- PATH OBJECTS ----
    private Path scorePreload;


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

        constants.shooter.TARGET_RPM = 900;
        constants.shooter.Hood_pos = 0.875;

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
        shooter.setHood(constants.shooter.Hood_pos);
        shooter.update();
    }


    // ---- BUILD PATHS ----
    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, parkpose));

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //Init
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

                        shootTimer.reset();
                        shotWaitStarted = true;
                    }

                    if (shootTimer.seconds() >= 2.0) {

                        intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                        shotWaitStarted = false;   // reset for next time
                        setPathState(99);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(scorePreload, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
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