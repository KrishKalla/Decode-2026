package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "shooter testing")
@Config
public class shootertesting extends OpMode {
    private boolean automatedDrive;
    private shooter shooter;
    private intake intake;
    ;
    public static double MANUALHOOD = 0.2;

    private int alliance = 1;
    private LLHandler llhandler;

    public void init() {
        shooter = new shooter();
        intake = new intake();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);
    }

    @Override
    public void start() {
        ;
    }

    public void loop() {
        intake.stallDetection();
        shooter.setHood(MANUALHOOD);
        shooter.updateBatteryVoltage();
        double shit = shooter.calculate();
        shooter.motorLeft.setPower(shit);
        shooter.motorRight.setPower(shit);
        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
        }
        if (gamepad1.dpad_down) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }
        if(gamepad1.right_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
            shooter.hoodPreset(constants.HOOD.MANUAL);
        }
        if(gamepad1.left_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.OFF);
            shooter.hoodPreset(constants.HOOD.RESET);
        }
        if (gamepad1.dpad_left) {
            shooter.setStopper(false);
        }
        if (gamepad1.dpad_right) {
            shooter.setStopper(true);
        }
        telemetry.addLine(shooter.toString());
        telemetry.addData("RPM: ", shooter.getRPM());
        telemetry.addData("POWER: ", shit);
        telemetry.update();
    }
}
