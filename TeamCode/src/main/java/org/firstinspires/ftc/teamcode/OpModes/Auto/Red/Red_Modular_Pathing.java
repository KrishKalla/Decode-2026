package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.constants;

/**
 * AutoLibrary
 *
 * ═════════════════════════════════════════════════════════════════════════════
 * ONE FILE – TWO SECTIONS
 * ═════════════════════════════════════════════════════════════════════════════
 *
 *  SECTION 1 – POSES  (static constants at the top)
 *  ─────────────────────────────────────────────────
 *  Every field position used across all autonomous routines lives here as a
 *  public static final Pose.  This is the ONLY place you touch when tuning
 *  field coordinates.
 *
 *  SECTION 2 – MODULES  (instance methods below)
 *  ─────────────────────────────────────────────────
 *  Each public method is one self-contained "module" – a full path + action
 *  sequence (e.g. drive to pickup → intake → return → shoot).
 *  Modules return true when they are finished so the calling OpMode can
 *  advance its state machine with a single if-statement.
 *
 * ═════════════════════════════════════════════════════════════════════════════
 * QUICK-START IN AN OPMODE
 * ═════════════════════════════════════════════════════════════════════════════
 *
 *  // 1. Construct once (pass subsystems + timing config)
 *  AutoLibrary auto = new AutoLibrary(follower, intake, shooter,
 *                                     SHOOT_DWELL_S, GATE_DWELL_S);
 *
 *  // 2. Build all paths (call after follower.setPose in init)
 *  auto.buildPaths();
 *
 *  // 3. Drive the state machine – one module per case
 *  switch (pathState) {
 *      case 0: if (auto.modFirstDriveAndShoot())        pathState = 1; break;
 *      case 1: if (auto.modNearIntakeAndShoot())         pathState = 2; break;
 *      case 2: if (auto.modGateIntakeAndShoot())         pathState = 3; break;
 *      case 3: if (auto.modCenterIntakeAndShoot())       pathState = 4; break;
 *      case 4: if (auto.modGateIntakeAndShoot())         pathState = 5; break;
 *      case 5: if (auto.modFarIntakeAndShoot())          pathState = 6; break;
 *      case 6: if (auto.modFinalGateIntakeParkGateAndShoot()) pathState = 7; break;
 *      case 7: auto.modShutdown(); requestOpModeStop();          break;
 *  }
 */
public class Red_Modular_Pathing {

    // ══════════════════════════════════════════════════════════════════════════
    //
    //  SECTION 1 – POSES
    //  Edit ONLY this section when tuning field positions.
    //  Coordinate system: Pedro Pathing field frame (inches, 0–144 × 0–144).
    //
    // ══════════════════════════════════════════════════════════════════════════

    // ── Start ─────────────────────────────────────────────────────────────────
    public static final Pose START              = new Pose(115.23, 125.38, Math.toRadians(0));

    // ── Score positions ───────────────────────────────────────────────────────
    /** First shot of the auto – robot is still far from goal, wider angle. */
    public static final Pose FIRST_SCORE        = new Pose(93,     85,    Math.toRadians(0));
    /** Standard score position used for every mid-routine shot. */
    public static final Pose SCORE              = new Pose(86,     76,    Math.toRadians(0));

    // ── Near (spike-mark / first row) pickup ──────────────────────────────────
    public static final Pose NEAR_PICKUP        = new Pose(120,    61,    Math.toRadians(0));
    /** Bezier control point: SCORE → NEAR_PICKUP. */
    public static final Pose MID_NEAR_PICKUP    = new Pose(88.2,   60,    Math.toRadians(0));

    // ── Center row pickup (second row) ────────────────────────────────────────
    public static final Pose CENTER_PICKUP      = new Pose(120,    80,    Math.toRadians(0));
    /** Bezier control point: SCORE → CENTER_PICKUP. */
    public static final Pose MID_CENTER_PICKUP  = new Pose(90,     84,    Math.toRadians(0));

    // ── Far row pickup (third row) ────────────────────────────────────────────
    public static final Pose FAR_PICKUP         = new Pose(120,    40,    Math.toRadians(0));
    /** Bezier control point: SCORE → FAR_PICKUP. */
    public static final Pose MID_FAR_PICKUP     = new Pose(86.271, 36,    Math.toRadians(0));

    // ── Gate intake ───────────────────────────────────────────────────────────
    public static final Pose GATE               = new Pose(131,    58.5,  Math.toRadians(22));
    /** Bezier control point: SCORE → GATE. */
    public static final Pose MID_GATE           = new Pose(106,    60,    Math.toRadians(0));

    // ── Park positions ────────────────────────────────────────────────────────
    /** Safe / default park near the audience wall. */
    public static final Pose PARK_SAFE          = new Pose(83.128, 103,   Math.toRadians(-45));
    /** Far-corner park – used when finishing on the far-row side. */
    public static final Pose PARK_FAR           = new Pose(83.128, 40,    Math.toRadians(-45));
    /** Gate-wall park – used when finishing after a gate intake. */
    public static final Pose PARK_GATE          = new Pose(120,    58.5,  Math.toRadians(0));


    // ══════════════════════════════════════════════════════════════════════════
    //
    //  SECTION 2 – MODULES
    //  Do not edit unless you are adding a new module or changing logic.
    //  To change field positions, edit the poses in Section 1 above.
    //
    // ══════════════════════════════════════════════════════════════════════════

    // ── Shared subsystem references ───────────────────────────────────────────
    Follower follower;
    intake   intake;
    shooter  shooter;

    // ── Timing config (set via constructor) ───────────────────────────────────
    /** Seconds the robot dwells at score pose before firing. */
    private static double shootingSeconds;
    /** Seconds the robot waits at the gate while collecting rings. */
    private static double gateIntakeSeconds;

    // ── Internal state ────────────────────────────────────────────────────────
    private final ElapsedTime dwellTimer   = new ElapsedTime();
    private boolean           dwellStarted = false;
    private boolean           returnStarted = false;

    // ── Pre-built path chains (naming: path_FROM_TO) ──────────────────────────
    private PathChain path_start_firstScore;

    private PathChain path_score_nearPickup;
    private PathChain path_nearPickup_score;

    private PathChain path_score_centerPickup;
    private PathChain path_centerPickup_score;

    private PathChain path_score_farPickup;
    private PathChain path_farPickup_score;

    private PathChain path_score_gate;
    private PathChain path_gate_score;

    private PathChain path_gate_parkSafe;
    private PathChain path_gate_parkFar;
    private PathChain path_gate_parkGate;

    private PathChain path_centerPickup_parkSafe;
    private PathChain path_centerPickup_parkFar;

    private PathChain path_farPickup_parkSafe;
    private PathChain path_farPickup_parkFar;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param follower          Pedro Pathing follower (already set to START pose)
     * @param intake            Intake subsystem
     * @param shooter           Shooter subsystem
     * @param shootDwellSeconds Seconds to pause at score pose before firing
     * @param gateDwellSeconds  Seconds to collect rings at the gate
     */
    public Red_Modular_Pathing(Follower follower,
                       intake   intake,
                       shooter  shooter,
                       double   shootDwellSeconds,
                       double   gateDwellSeconds) {
        this.follower          = follower;
        this.intake            = intake;
        this.shooter           = shooter;
        this.shootingSeconds = shootDwellSeconds;
        this.gateIntakeSeconds  = gateDwellSeconds;
    }

    // ── buildPaths() – call once in OpMode.init() after follower.setPose() ────

    public void buildPaths() {

        path_start_firstScore = follower.pathBuilder()
                .addPath(new BezierLine(START, FIRST_SCORE))
                .setLinearHeadingInterpolation(START.getHeading(), SCORE.getHeading())
                .build();

        // Score ↔ Near Pickup
        path_score_nearPickup = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE, MID_NEAR_PICKUP, NEAR_PICKUP))
                .setLinearHeadingInterpolation(SCORE.getHeading(), NEAR_PICKUP.getHeading())
                .build();
        path_nearPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(NEAR_PICKUP, SCORE))
                .setLinearHeadingInterpolation(NEAR_PICKUP.getHeading(), SCORE.getHeading())
                .build();

        // Score ↔ Center Pickup
        path_score_centerPickup = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE, MID_CENTER_PICKUP, CENTER_PICKUP))
                .setLinearHeadingInterpolation(SCORE.getHeading(), CENTER_PICKUP.getHeading())
                .build();
        path_centerPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(CENTER_PICKUP, SCORE))
                .setLinearHeadingInterpolation(CENTER_PICKUP.getHeading(), SCORE.getHeading())
                .build();

        // Score ↔ Far Pickup
        path_score_farPickup = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE, MID_FAR_PICKUP, FAR_PICKUP))
                .setLinearHeadingInterpolation(SCORE.getHeading(), FAR_PICKUP.getHeading())
                .build();
        path_farPickup_score = follower.pathBuilder()
                .addPath(new BezierLine(FAR_PICKUP, SCORE))
                .setLinearHeadingInterpolation(FAR_PICKUP.getHeading(), SCORE.getHeading())
                .build();

        // Score ↔ Gate
        path_score_gate = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE, MID_GATE, GATE))
                .setLinearHeadingInterpolation(SCORE.getHeading(), GATE.getHeading())
                .build();
        path_gate_score = follower.pathBuilder()
                .addPath(new BezierLine(GATE, SCORE))
                .setLinearHeadingInterpolation(GATE.getHeading(), SCORE.getHeading())
                .build();

        // Gate → Park (all variants)
        path_gate_parkSafe = follower.pathBuilder()
                .addPath(new BezierLine(GATE, PARK_SAFE))
                .setLinearHeadingInterpolation(GATE.getHeading(), PARK_SAFE.getHeading())
                .build();
        path_gate_parkFar = follower.pathBuilder()
                .addPath(new BezierLine(GATE, PARK_FAR))
                .setLinearHeadingInterpolation(GATE.getHeading(), PARK_FAR.getHeading())
                .build();
        path_gate_parkGate = follower.pathBuilder()
                .addPath(new BezierLine(GATE, PARK_GATE))
                .setLinearHeadingInterpolation(GATE.getHeading(), PARK_GATE.getHeading())
                .build();

        // Center Pickup → Park
        path_centerPickup_parkSafe = follower.pathBuilder()
                .addPath(new BezierLine(CENTER_PICKUP, PARK_SAFE))
                .setLinearHeadingInterpolation(CENTER_PICKUP.getHeading(), PARK_SAFE.getHeading())
                .build();
        path_centerPickup_parkFar = follower.pathBuilder()
                .addPath(new BezierLine(CENTER_PICKUP, PARK_FAR))
                .setLinearHeadingInterpolation(CENTER_PICKUP.getHeading(), PARK_FAR.getHeading())
                .build();

        // Far Pickup → Park
        path_farPickup_parkSafe = follower.pathBuilder()
                .addPath(new BezierLine(FAR_PICKUP, PARK_SAFE))
                .setLinearHeadingInterpolation(FAR_PICKUP.getHeading(), PARK_SAFE.getHeading())
                .build();
        path_farPickup_parkFar = follower.pathBuilder()
                .addPath(new BezierLine(FAR_PICKUP, PARK_FAR))
                .setLinearHeadingInterpolation(FAR_PICKUP.getHeading(), PARK_FAR.getHeading())
                .build();
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  FIRST MODULE
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Module: Drive from START to FIRST_SCORE and shoot preloaded rings.
     * Returns true when the shot is away and the robot is ready for the next module.
     */
    public boolean modFirstDriveAndShoot() {
        if (!follower.isBusy() && !dwellStarted) {
            follower.setMaxPower(0.9);
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
            shooter.setStopper(false);
            follower.followPath(path_start_firstScore, false);
            dwellStarted = true;
            return false;
        }
        if (!follower.isBusy()) {
            return performShootDwell(() -> {
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                resetDwell();
            });
        }
        return false;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  MID-ROUTINE MODULES  (score → pickup → score → shoot)
    // ══════════════════════════════════════════════════════════════════════════

    /** Module: Near (spike-mark / first row) intake → return to score → shoot. */
    public boolean modNearIntakeAndShoot() {
        return intakePickupCycle(path_score_nearPickup, path_nearPickup_score);
    }

    /** Module: Center row intake → return to score → shoot. */
    public boolean modCenterIntakeAndShoot() {
        return intakePickupCycle(path_score_centerPickup, path_centerPickup_score);
    }

    /** Module: Far row intake → return to score → shoot. */
    public boolean modFarIntakeAndShoot() {
        return intakePickupCycle(path_score_farPickup, path_farPickup_score);
    }

    /**
     * Module: Gate intake → dwell at gate → return to score → shoot.
     * The robot waits gateDwellSeconds at the gate before returning.
     */
    public boolean modGateIntakeAndShoot() {
        // Phase A: drive to gate
        if (!follower.isBusy() && !dwellStarted) {
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
            follower.followPath(path_score_gate, false);
            dwellStarted = true;
            return false;
        }
        // Phase B: dwell at gate
        if (!follower.isBusy() && !returnStarted) {
            if (!isTimerRunning()) {
                follower.setMaxPower(0.2);
                dwellTimer.reset();
            }
            if (dwellTimer.seconds() < gateIntakeSeconds) return false;
            // Gate dwell done – return to score
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
            follower.followPath(path_gate_score, false);
            returnStarted = true;
            return false;
        }
        // Phase C: back at score – shoot dwell
        if (!follower.isBusy() && returnStarted) {
            return performShootDwell(() -> {
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                resetDwell();
            });
        }
        return false;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  FINAL MODULES  (last pickup + park + final shot)
    //  Three variants – pick the one that matches where your robot ends up.
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Module: Intake CENTER row → drive to SAFE park → final shot.
     * Best for routines that finish near the audience wall.
     */
    public boolean modFinalCenterIntakeParkSafeAndShoot() {
        return finalPickupParkShoot(path_score_centerPickup, path_centerPickup_parkSafe);
    }

    /**
     * Module: Intake FAR row → drive to FAR park → final shot.
     * Best for routines that finish in the far-field corner.
     */
    public boolean modFinalFarIntakeParkFarAndShoot() {
        return finalPickupParkShoot(path_score_farPickup, path_farPickup_parkFar);
    }

    /**
     * Module: Gate intake → dwell → drive to GATE-WALL park → final shot.
     * Best for routines that finish near the gate wall.
     */
    public boolean modFinalGateIntakeParkGateAndShoot() {
        // Phase A: drive to gate
        if (!follower.isBusy() && !dwellStarted) {
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
            follower.followPath(path_score_gate, false);
            dwellStarted = true;
            return false;
        }
        // Phase B: dwell at gate
        if (!follower.isBusy() && !returnStarted) {
            if (!isTimerRunning()) {
                follower.setMaxPower(0.2);
                dwellTimer.reset();
            }
            if (dwellTimer.seconds() < gateIntakeSeconds) return false;
            // Gate dwell done – drive to park
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
            follower.followPath(path_gate_parkGate, false);
            returnStarted = true;
            return false;
        }
        // Phase C: at park – final shoot dwell
        if (!follower.isBusy() && returnStarted) {
            return performShootDwell(() -> {
                intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                shooter.setStopper(true);
                resetDwell();
            });
        }
        return false;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  SHUTDOWN MODULE
    // ══════════════════════════════════════════════════════════════════════════

    /** Turn off all subsystems. Call immediately before requestOpModeStop(). */
    public void modShutdown() {
        intake.setIntake(constants.INTAKE_PRESETS.OFF);
        shooter.flywheelPreset(constants.FLYWHEEL.OFF);
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  PRIVATE HELPERS
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Standard mid-routine pickup cycle:
     *   Drive to pickup (intake ON) → arrive → drive back to score → shoot dwell.
     */
    private boolean intakePickupCycle(PathChain toPickup, PathChain toScore) {
        // Phase A: drive to pickup
        if (!follower.isBusy() && !dwellStarted) {
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
            follower.followPath(toPickup, false);
            dwellStarted = true;
            return false;
        }
        // Phase B: arrived at pickup – immediately drive back
        if (!follower.isBusy() && !returnStarted) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
            follower.followPath(toScore, false);
            returnStarted = true;
            return false;
        }
        // Phase C: back at score – shoot dwell
        if (!follower.isBusy() && returnStarted) {
            return performShootDwell(() -> {
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
                resetDwell();
            });
        }
        return false;
    }

    /**
     * Final pickup + direct-to-park cycle (no return to score):
     *   Drive to pickup (intake ON) → arrive → drive to park → final shoot dwell.
     */
    private boolean finalPickupParkShoot(PathChain toPickup, PathChain toPark) {
        // Phase A: drive to pickup
        if (!follower.isBusy() && !dwellStarted) {
            follower.setMaxPower(1.0);
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
            follower.followPath(toPickup, false);
            dwellStarted = true;
            return false;
        }
        // Phase B: arrived at pickup – drive to park
        if (!follower.isBusy() && !returnStarted) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
            follower.followPath(toPark, false);
            returnStarted = true;
            return false;
        }
        // Phase C: at park – final shoot dwell
        if (!follower.isBusy() && returnStarted) {
            return performShootDwell(() -> {
                intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
                shooter.setStopper(true);
                resetDwell();
            });
        }
        return false;
    }

    /** Slows the robot, waits shootDwellSeconds, then calls onComplete and returns true. */
    private boolean performShootDwell(Runnable onComplete) {
        if (!isTimerRunning()) {
            follower.setMaxPower(0.2);
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
            dwellTimer.reset();
        }
        if (dwellTimer.seconds() < shootingSeconds) return false;
        follower.setMaxPower(1.0);
        onComplete.run();
        return true;
    }

    /** Returns true only after dwellTimer has been started this module invocation. */
    private boolean isTimerRunning() {
        return dwellStarted && dwellTimer.seconds() < 3600;
    }

    /** Resets all internal module state. Called automatically at end of each module. */
    private void resetDwell() {
        dwellStarted  = false;
        returnStarted = false;
    }

    /** Public reset – call this if you need to force a clean state between modules. */
    public void reset() {
        resetDwell();
    }
}