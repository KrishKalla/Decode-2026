package org.firstinspires.ftc.teamcode.OpModes.Auto.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Auto.Red.Red_Modular_Pathing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

/**
 * Red 21 Safe – uses AutoLibrary for all poses and modules.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * SEQUENCE  (change only the switch cases below to build a different auto)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  State  0  → First drive + shoot preloaded rings
 *  State  1  → Near (spike-mark) intake + return + shoot
 *  State  2  → Gate intake #1 + return + shoot
 *  State  3  → Center row intake + return + shoot
 *  State  4  → Gate intake #2 + return + shoot
 *  State  5  → Far row intake + return + shoot
 *  State  6  → FINAL: gate intake #3 → park (safe) → shoot
 *  State  7  → Shutdown + stop
 *
 * To build a DIFFERENT auto, copy this file, rename the class + @Autonomous,
 * then rewire the cases.  Every module is one method call.
 */
@Config
@Autonomous(name = "Red 21 Auto - Safe (Modular)")
public class Red_testing extends OpMode {

    // ── Tunable timing (editable live via FTC Dashboard) ──────────────────
    public static double SHOOT_DWELL_S = 0.45;   // seconds to pause before shooting
    public static double GATE_DWELL_S  = 1.30;   // seconds to collect at gate

    // ── Subsystems ────────────────────────────────────────────────────────
    private Follower  follower;
    private intake    intake;
    private shooter   shooter;
    private turret    turret;
    private LLHandler llhandler;

    // ── Modules + timing ──────────────────────────────────────────────────
    private Red_Modular_Pathing auto;
    private ElapsedTime runtime = new ElapsedTime();

    // ── State machine ─────────────────────────────────────────────────────
    private int pathState = 0;

    // ── Goal for turret ───────────────────────────────────────────────────
    private final com.pedropathing.geometry.Pose goalPose =
            new com.pedropathing.geometry.Pose(storage.RED_X, storage.RED_Y);

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        shooter  = new shooter();
        intake   = new intake();
        turret   = new turret();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Red_Modular_Pathing.START);

        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        constants.shooter.TARGET_RPM   = 1440;
        constants.shooter.Target_Hood  = 0.64;

        auto = new Red_Modular_Pathing(follower, intake, shooter, SHOOT_DWELL_S, GATE_DWELL_S);
        auto.buildPaths();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        runStateMachine();
        shooter.update();
        intake.update();
        turret.update(goalPose);
        turret.periodic();

        if (!follower.getPose().equals(new com.pedropathing.geometry.Pose(0, 0, 0))) {
            storage.lastRedAutoPose = follower.getPose();
        }

        telemetry.addData("State",           pathState);
        telemetry.addData("X",               follower.getPose().getX());
        telemetry.addData("Y",               follower.getPose().getY());
        telemetry.addData("Heading (deg)",   Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("RPM",             shooter.getRPM());
        telemetry.addData("Time",            runtime.seconds());
        telemetry.addData("Turret Target",   turret.getTargetAngle());
        telemetry.addData("Turret Current",  turret.getCurrentAngle());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────
    //  STATE MACHINE
    //  ► Each case is exactly ONE module call.
    //  ► When the module returns true it is finished; advance pathState.
    //  ► To build a new auto: copy, rename, and swap module calls here.
    // ─────────────────────────────────────────────────────────────────────
    private void runStateMachine() {
        switch (pathState) {

            case 0:
                if (auto.modFirstDriveAndShoot())               pathState = 1;
                break;

            case 1:
                constants.shooter.TARGET_RPM   = 1500;
                constants.shooter.Target_Hood  = 0.69;
                if (auto.modNearIntakeAndShoot())                pathState = 2;
                break;

            case 2:
                if (auto.modGateIntakeAndShoot())                pathState = 3;
                break;

            case 3:
                if (auto.modCenterIntakeAndShoot())              pathState = 4;
                break;

            case 4:
                if (auto.modGateIntakeAndShoot())                pathState = 5;
                break;

            case 5:
                if (auto.modFarIntakeAndShoot())                 pathState = 6;
                break;

            case 6:
                // ── Swap for a different ending: ─────────────────────────────
                //   auto.modFinalCenterIntakeParkSafeAndShoot()
                //   auto.modFinalFarIntakeParkFarAndShoot()
                //   auto.modFinalGateIntakeParkGateAndShoot()
                if (auto.modFinalGateIntakeParkGateAndShoot())   pathState = 7;
                break;

            case 7:
                auto.modShutdown();
                requestOpModeStop();
                break;
        }
    }
}