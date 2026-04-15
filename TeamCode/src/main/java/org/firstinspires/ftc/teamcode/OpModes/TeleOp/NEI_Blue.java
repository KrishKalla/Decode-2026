package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.SOTM;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

@Config
@TeleOp(name = "Blue Close TeleOp ", group = "1")
public class NEI_Blue extends OpMode {
    private Follower follower;
    private intake intake;
    private turret turret;
    private shooter shooter;
    private LLHandler llhandler;

    private ElapsedTime timer;
    private ElapsedTime threadTimer = new ElapsedTime();
    private final int alliance = 1;

    Thread thread;
    Runnable r;

    private volatile double servoUpdate;

    public static double RPM_Constraint = 1560;
    public static double  Dist_offset= 0.075;
    public static boolean AUTO = true;
    public static boolean AUTO_AIM = true;
    private Pose goalpose = new Pose(storage.BLUE_X, storage.BLUE_Y);
    private Pose SOTMpose = new Pose(storage.BLUE_X, storage.BLUE_Y);

    private int Mode = 0; // Short

    // Heading Lock
    double targetHeading = Math.toRadians(157);
    public static double heading_P = 0.3;
    public static double heading_D = 0;
    public static double heading_F = 0;
    public static PIDFController controller = new PIDFController(new PIDFCoefficients(heading_P, 0, heading_D, heading_F));
    public static boolean headingLock = false;

    // ── Auto Drive-to-Pose ────────────────────────────────────────────────────
    // Tune these from FTC Dashboard while on the field
    public static double AUTO_DRIVE_X       = 13;
    public static double AUTO_DRIVE_Y       = 58;
    public static double AUTO_DRIVE_HEADING = 157; // degrees, converted to radians on use

    public static double AUTO_DRIVE_X2       = 64;
    public static double AUTO_DRIVE_Y2       = 80;
    public static double AUTO_DRIVE_HEADING2 = 180;

    private boolean automatedDrive = false;
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(storage.lastBlueAutoPose);

        follower.update();

        intake  = new intake();
        turret  = new turret();
        shooter = new shooter();
        llhandler = new LLHandler(hardwareMap, alliance);
        llhandler.alliance(alliance);

        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);
        shooter.init(hardwareMap, llhandler);

        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        r = new Runnable() {
            @Override
            public void run() {
                while (true) {
                    threadTimer.reset();
                    llhandler.poll();
                    shooter.update();
                    shooter.updateBatteryVoltage();
                    if(Math.abs(follower.getVelocity().getXComponent() + follower.getVelocity().getYComponent())<=10){
                        shooter.calculateParams(RPM_Constraint,0);
                    }
                    else if (AUTO){
                        shooter.calculateParams(RPM_Constraint, turret.SOTM_dist_BLUE(SOTM.getAdjustedGoal())+Dist_offset);
                    }
                    else{
                        shooter.calculateParams(RPM_Constraint,0);
                    }

                }
            }
        };

        thread = new Thread(r);
    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        llhandler.start();
        intake.setIntake(constants.INTAKE_PRESETS.OFF);

        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        shooter.hoodPreset(constants.HOOD.AUTO);
        AUTO     = true;
        AUTO_AIM = true;
        shooter.setStopper(true);

        thread.start();
        timer.reset();

        SOTM.setGoalPose(goalpose);
    }

    @Override
    public void loop() {

        if (follower.getPose().getY()>=48){
            goalpose=new Pose(storage.BLUE_X-2,storage.BLUE_Y);
        }
        else{
            goalpose=new Pose(storage.BLUE_X,storage.BLUE_Y);
        }
        SOTM.setGoalPose(goalpose);

        // ── Auto Drive-to-Pose trigger ────────────────────────────────────────
        // gamepad2.options  → start following a path to the dashboard-configured pose
        // gamepad2.share    → cancel and return to teleop drive immediately
        if (gamepad1.square && !automatedDrive) {
            startAutoDrive_Gate();
        }

        if (gamepad1.triangle && !automatedDrive) {
            startAutoDrive_Shoot();
        }

        if (gamepad1.circle && automatedDrive) {
            cancelAutoDrive();
        }

        // When following a path, check if it's finished
        if (automatedDrive && !follower.isBusy()) {
            cancelAutoDrive(); // path complete → hand control back
        }
        // ─────────────────────────────────────────────────────────────────────

        controller = new PIDFController(new PIDFCoefficients(heading_P, 0, heading_D, heading_F));
        controller.updateError(getHeadingError());

        // Only accept manual drive input when not in auto-drive mode
        if (!automatedDrive) {
            if (headingLock)
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(), true);
            else
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.55, true);
        }

        intake.update();
        timer.reset();
        follower.update();

        if (AUTO_AIM) {
            SOTM.calculate(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
            turret.update(SOTM.getAdjustedGoal());
            turret.periodic();
        }

        // ── Intake controls (skipped during auto-drive — handled automatically) ──
        if (!automatedDrive) {
            if (intake.blocked) {
                gamepad1.rumble(100);
            } else {
                gamepad1.stopRumble();
            }
            if (gamepad1.right_stick_button) {
                intake.setIntake(constants.INTAKE_PRESETS.GATE_BLUE);
            } else if (gamepad1.right_trigger > 0.3) {
                intake.setIntake(constants.INTAKE_PRESETS.ON);
                shooter.setStopper(true);
            } else if (gamepad1.left_trigger > 0.3) {
                intake.setIntake(constants.INTAKE_PRESETS.REJECT);
                intake.blocked = false;
            } else if (gamepad1.right_bumper) {
                shooter.setStopper(false);
                intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
            } else {
                intake.setIntake(constants.INTAKE_PRESETS.OFF);
            }
        }

        // Open stopper
        if (gamepad1.left_bumper) {
            shooter.setStopper(false);
        }

        // Switch modes / heading lock
        if (gamepad1.right_stick_button) {
            targetHeading = Math.toRadians(158);
            headingLock   = true;
            shooter.setStopper(true);
        } else if (gamepad2.right_trigger > 0.3) {
            targetHeading = Math.toRadians(0);
            headingLock   = true;
        } else {
            headingLock = false;
        }

        // Human Player Zone reloc
        if (gamepad2.right_stick_button) {
            follower.setY(7.8);
        }
        if (gamepad2.left_stick_button){
            follower.setHeading(0);
            follower.setX(136);
        }

        // Close Zone set-points
        if (gamepad2.square) {
            AUTO = false;
            constants.shooter.TARGET_RPM = 1300;
            constants.shooter.Hood_pos   = 0.291;
        }
        if (gamepad2.triangle) {
            AUTO = false;
            constants.shooter.TARGET_RPM = 1420;
            constants.shooter.Hood_pos   = 0.5075;
        }
        if (gamepad2.circle) {
            AUTO = false;
            constants.shooter.TARGET_RPM = 1800;
            constants.shooter.Hood_pos   = 0.74;
        }
        if (gamepad2.cross) {
            AUTO = false;
        }

        // Manual turret Offset
        if (gamepad2.dpad_left) {
            constants.shooter.Goal_delta++;
        }
        if (gamepad2.dpad_right) {
            constants.shooter.Goal_delta--;
        }

        // Close zone
        if (gamepad2.dpad_up) {
            constants.intake.TRANSFER_POWER=1;
            RPM_Constraint=1560;
        }
        // Far zone
        if (gamepad2.dpad_down) {
            constants.intake.TRANSFER_POWER=0.67;
            RPM_Constraint=2000;
        }

        updateTelemetry();
    }

    /** Builds a path from the current pose to the dashboard-configured target and starts following it. */
    private void startAutoDrive_Gate() {
        Pose target = new Pose(AUTO_DRIVE_X, AUTO_DRIVE_Y, Math.toRadians(AUTO_DRIVE_HEADING));

        PathChain chain = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, target)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading,
                                Math.toRadians(AUTO_DRIVE_HEADING), 0.8))
                .build();

        follower.followPath(chain, true); // true = hold end position
        automatedDrive = true;

        // Turn intake on and open stopper for the ride
        intake.setIntake(constants.INTAKE_PRESETS.GATE_BLUE);
        shooter.setStopper(true);
    }
    private void startAutoDrive_Shoot() {
        Pose target = new Pose(AUTO_DRIVE_X2, AUTO_DRIVE_Y2, Math.toRadians(AUTO_DRIVE_HEADING2));

        PathChain chain = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, target)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading,
                                Math.toRadians(AUTO_DRIVE_HEADING2), 0.8))
                .build();

        follower.followPath(chain, true);
        automatedDrive = true;

        intake.setIntake(constants.INTAKE_PRESETS.OFF);
        shooter.setStopper(false);
    }

    /** Cancels an in-progress auto-drive and hands control back to the driver. */
    private void cancelAutoDrive() {
        follower.startTeleopDrive(); // re-enable driver input
        automatedDrive = false;

        // Reset intake/stopper to safe defaults
        intake.setIntake(constants.INTAKE_PRESETS.OFF);
        shooter.setStopper(false);
    }


    public void updateTelemetry() {
        telemetry.addLine(follower.getPose().toString());
        telemetry.addData("Turret_delta",-constants.shooter.Goal_delta);

        telemetry.addData("X-V-Vector",follower.getVelocity().getXComponent());
        telemetry.addData("Y-V-Vector",follower.getVelocity().getYComponent());

        telemetry.addLine("≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡ROBOT≡≡≡≡≡≡≡≡≡≡≡≡≡≡");
        telemetry.addData("AUTO AIM ACTIVE",    AUTO_AIM);
        telemetry.addData("REGRESSION ACTIVE",  AUTO);// new
        telemetry.addData("EMA",                shooter.ema);
        telemetry.addData("RPM",                shooter.getRPM());
        telemetry.addData("Hood Angle",         shooter.getHoodAngle());
        telemetry.addData("Current Heading",    Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public double getHeadingError() {
        return MathFunctions.getTurnDirection(follower.getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }

    @Override
    public void stop() {
        thread.interrupt();
    }
}