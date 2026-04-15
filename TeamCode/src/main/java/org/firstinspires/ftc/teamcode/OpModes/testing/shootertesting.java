package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "shooter testing", group = "testing")
@Config
public class shootertesting extends OpMode {
    public static boolean ON = false;
    public static boolean Transfering = false;
    public static boolean Stopper = false;
    public static boolean Intake = false;
    private boolean automatedDrive;
    private shooter shooter;
    private intake intake;
    private turret turret;
    private Follower follower;
    public static double MANUAL_Turret =0;

    public static int alliance = 1;
    private LLHandler llhandler;
    private final Pose goalPose = new Pose(storage.RED_X, storage.RED_Y);

    public void init() {
        shooter = new shooter();
        intake = new intake();
        turret = new turret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(storage.lastRedAutoPose);
        follower.update();

        turret.init(hardwareMap, follower);
    }

    @Override
    public void start() {
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        llhandler.alliance(alliance);
        llhandler.start();
    }

    public void loop() {
        follower.update();
        llhandler.poll();
        shooter.update();
        shooter.updateBatteryVoltage();

        if (ON) {
            turret.update(goalPose);
            turret.periodic();
        } else {
            turret.update(MANUAL_Turret);
            turret.periodic();
        }

        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
        }

        if (gamepad1.left_bumper) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
        }

        if (gamepad1.right_bumper) {
            shooter.setStopper(true);
        }

        if (gamepad1.cross) {
            shooter.setStopper(false);
        }

        telemetry.addLine(shooter.toString());
        telemetry.addData("RPM: ", shooter.getRPM());
        telemetry.addData("Power:", shooter.getPower());
        telemetry.addData("HOOD POS", shooter.getHoodAngle());
        telemetry.addData("Target_RPM: ", constants.shooter.TARGET_RPM);
        telemetry.addData("Target_Hood: ", constants.shooter.Hood_pos);
        telemetry.addData("lldist converted", Math.sqrt(Math.pow(llhandler.getLatestResult()[2], 2) - Math.pow(constants.APRIL_TAG_HEIGHT - constants.LIMELIGHT_HEIGHT, 2)));
        telemetry.addData("Turret Angle", turret.getCurrentAngle());
        telemetry.addData("Turret Target", turret.getTargetAngle());
        telemetry.addData("Turret Error", turret.getError());

        telemetry.addData("Mode", ON ? "AUTO" : "MANUAL");
        telemetry.addLine(intake.toString());
        telemetry.update();
    }
}