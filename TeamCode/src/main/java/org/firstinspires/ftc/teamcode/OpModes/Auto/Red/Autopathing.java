package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

/**
 * AutoPathing — base class for all autonomous OpModes.
 *
 * Handles:
 *   - Subsystem init
 *   - All shared path declarations and buildPaths()
 *   - shootWhileMoving() opening action
 *   - rowIntakeAndScore() and gateIntakeAndScore() action functions
 *   - setPathState() utility
 *
 * In your auto OpMode, just extend this, override getStartPose(), and write your sequence.
 */
public abstract class Autopathing extends OpMode {

    // =========================================================
    // TUNABLE CONSTANTS — override in subclass if needed
    // =========================================================
    public double SHOOT_TIME        = 0.45;
    public double GATE_INTAKE_TIME  = 1.5;
    public double SHOOTING_MOMENT   = 1.4;   // when to fire during shootWhileMoving
    public double SHOOTING_HOOD     = 0.35;
    public double OPENING_POWER     = 0.5;   // drive power during shootWhileMoving
    public double turret_offset     = 0;

    // =========================================================
    // SUBSYSTEMS — shared across all autos
    // =========================================================
    public Follower  follower;
    public intake    intake;
    public shooter   shooter;
    public Turret    turret;
    public LLHandler llhandler;

    public Pose goalPose;

    // =========================================================
    // STATE
    // =========================================================
    public int subState  = 0;
    public int pathState = 0;

    public ElapsedTime shootTimer = new ElapsedTime();
    public ElapsedTime runtime    = new ElapsedTime();
    public ElapsedTime loopTimer  = new ElapsedTime();

    private boolean shotWaitStarted  = false;
    private boolean IsShot           = false;

    // =========================================================
    // POSES — shared field geometry, override in subclass if needed
    // =========================================================
    public Pose startPose        = new Pose(115.75, 126.79, Math.toRadians(0));
    public Pose scorePose        = new Pose(92,     76,     Math.toRadians(0));
    public Pose pickup1Pose      = new Pose(116,    59,     Math.toRadians(0));
    public Pose midPickup1       = new Pose(70,     55.5);
    public Pose gateApproachPose = new Pose(130.5,  61,     Math.toRadians(20));
    public Pose midcenterPickup  = new Pose(90,     88);
    public Pose centerPickupPose = new Pose(116,    84.5,   Math.toRadians(0));
    public Pose midFarPickup     = new Pose(86.271, 31.767);
    public Pose farPickupPose    = new Pose(116,    36,     Math.toRadians(0));
    public Pose parkPose         = new Pose(83.128, 103,    Math.toRadians(-45));

    // =========================================================
    // PATHS — declared here, built in buildPaths()
    // =========================================================
    public PathChain path_start_nearPickup;
    public PathChain path_nearPickup_score;
    public PathChain path_score_gate;
    public PathChain path_gate_score;
    public PathChain path_score_centerPickup;
    public PathChain path_centerPickup_score;
    public PathChain path_score_farPickup;
    public PathChain path_farPickup_score;
    public PathChain path_gate_park;

    // =========================================================
    // INIT — call super.init() in your subclass, then set any overrides
    // =========================================================
    @Override
    public void init() {
        shooter  = new shooter();
        intake   = new intake();
        turret   = new Turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);
        turret.zeroTurret();

        goalPose = new Pose(storage.RED_X, storage.RED_Y);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        constants.shooter.TARGET_RPM  = 780;
        constants.shooter.Target_Hood = SHOOTING_HOOD;

        buildPaths();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    // =========================================================
    // LOOP — call super.loop() in your subclass
    // =========================================================
    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        shooter.update();
        intake.update();

        turret.hardwareUpdate(turret.update(goalPose) + turret.angleToServoDelta(turret_offset));
        storage.lastRedAutoPose = follower.getPose();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Sub State",  subState);
        telemetry.addData("X",          follower.getPose().getX());
        telemetry.addData("Y",          follower.getPose().getY());
        telemetry.addData("Heading",    Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",        shooter.getRPM());
        telemetry.addData("Time",       runtime.seconds());
        telemetry.addData("Loop ms",    loopTimer.milliseconds());
        telemetry.update();
    }

    // =========================================================
    // BUILD PATHS — builds all shared paths from the poses above
    // Call buildPaths() at the end of your subclass init() if you
    // change any poses, so paths are rebuilt with the new values.
    // =========================================================
    public void buildPaths() {
        path_start_nearPickup = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, midPickup1, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        path_nearPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        path_score_gate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gateApproachPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateApproachPose.getHeading())
                .build();

        path_gate_score = follower.pathBuilder()
                .addPath(new BezierLine(gateApproachPose, scorePose))
                .setLinearHeadingInterpolation(gateApproachPose.getHeading(), scorePose.getHeading())
                .build();

        path_score_centerPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midcenterPickup, centerPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), centerPickupPose.getHeading())
                .build();

        path_centerPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(centerPickupPose, scorePose))
                .setLinearHeadingInterpolation(centerPickupPose.getHeading(), scorePose.getHeading())
                .build();

        path_score_farPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, midFarPickup, farPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), farPickupPose.getHeading())
                .build();

        path_farPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(farPickupPose, scorePose))
                .setLinearHeadingInterpolation(farPickupPose.getHeading(), scorePose.getHeading())
                .build();

        path_gate_park = follower.pathBuilder()
                .addPath(new BezierLine(gateApproachPose, parkPose))
                .setLinearHeadingInterpolation(parkPose.getHeading(), parkPose.getHeading())
                .build();
    }

    // =========================================================
    // ACTION: Shoot While Moving
    // =========================================================
    /**
     * Shoots the preloaded ring while driving to the near pickup row.
     * Call this as case 0 in your sequence — advances to pathState 1 when done.
     *
     * Uses OPENING_POWER, SHOOTING_MOMENT, and turret_offset from this class.
     * Override those in your subclass to tune per-auto.
     */
    public void shootWhileMoving() {
        if (!shotWaitStarted) {
            follower.setMaxPower(OPENING_POWER);
            follower.followPath(path_start_nearPickup, false);
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
            shooter.setStopper(false);
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shootTimer.reset();
            shotWaitStarted = true;
        }
        if (shootTimer.seconds() >= SHOOTING_MOMENT && !IsShot) {
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
        }
        if (shootTimer.seconds() >= SHOOTING_MOMENT + 0.4) {
            follower.setMaxPower(1);
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            constants.shooter.Target_Hood = 0.69;
            turret_offset   = 0;
            shooter.setStopper(true);
            IsShot          = true;
            shotWaitStarted = false;
            setPathState(1);
        }
    }

    // =========================================================
    // ACTION: Row Intake and Score
    // =========================================================
    /**
     * Drives to a field row pickup with intake on, then returns to score and shoots.
     *
     * @param toPickup   path to the pickup row
     * @param fromPickup path back to score position
     * @param nextState  pathState to advance to when complete
     *
     * sub 0 → drive to pickup, intake on
     * sub 1 → wait until arrived, drive back
     * sub 2 → wait until at score, start shoot timer
     * sub 3 → wait shoot timer, advance
     */
    public void rowIntakeAndScore(PathChain toPickup, PathChain fromPickup, int nextState) {
        switch (subState) {
            case 0:
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                follower.followPath(toPickup, false);
                subState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(fromPickup, false);
                    subState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.2);
                    intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    shootTimer.reset();
                    subState = 3;
                }
                break;

            case 3:
                if (shootTimer.seconds() >= SHOOT_TIME) {
                    follower.setMaxPower(1);
                    intake.setIntake(constants.INTAKE_PRESETS.ON);
                    shooter.setStopper(true);
                    setPathState(nextState);
                }
                break;
        }
    }
//    public void CloseRow(int nextState){
//        rowIntakeAndScore(path_score_centerPickup, path_centerPickup_score, nextState);
//    }
//    public void MiddleRow(int nextState){
//        rowIntakeAndScore(path_score_near, path_centerPickup_score, nextState);
//    }
//    public void CloseRow(int nextState){
//        rowIntakeAndScore(path_score_centerPickup, path_centerPickup_score, nextState);
//    }

    // =========================================================
    // ACTION: Gate Intake and Score
    // =========================================================
    /**
     * Drives to the gate with intake on, waits to collect balls, returns and shoots.
     *
     * @param toGate    path to gate approach position
     * @param fromGate  path back from gate (score or park)
     * @param nextState pathState to advance to when complete
     *
     * sub 0 → drive to gate, intake on
     * sub 1 → wait until arrived, slow down, start gate timer
     * sub 2 → wait at gate, then drive back
     * sub 3 → wait until arrived, start shoot timer
     * sub 4 → wait shoot timer, advance
     */
    public void gateIntakeAndScore(PathChain toGate, PathChain fromGate, int nextState) {
        switch (subState) {
            case 0:
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                follower.followPath(toGate, false);
                subState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.2);
                    shootTimer.reset();
                    subState = 2;
                }
                break;

            case 2:
                if (shootTimer.seconds() >= GATE_INTAKE_TIME) {
                    follower.setMaxPower(1);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(false);
                    follower.followPath(fromGate, false);
                    subState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.2);
                    intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                    shootTimer.reset();
                    subState = 4;
                }
                break;

            case 4:
                if (shootTimer.seconds() >= SHOOT_TIME) {
                    follower.setMaxPower(1);
                    intake.setIntake(constants.INTAKE_PRESETS.OFF);
                    shooter.setStopper(true);
                    setPathState(nextState);
                }
                break;
        }
    }

    // =========================================================
    // UTILS
    // =========================================================
    /**
     * Always use this to change pathState — resets subState automatically.
     */
    public void setPathState(int pState) {
        pathState = pState;
        subState  = 0;
    }
}