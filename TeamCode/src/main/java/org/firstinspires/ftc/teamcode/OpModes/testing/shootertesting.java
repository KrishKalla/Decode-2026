package org.firstinspires.ftc.teamcode.OpModes.testing;

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

@TeleOp(name = "total shooter testing")
@Config
public class shootertesting extends OpMode {
    private boolean automatedDrive;
    private shooter shooter;

    public static double MANUALHOOD = 0.165;

    private int alliance = 1;
    private LLHandler llhandler;

    public void init() {
        shooter = new shooter();
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
    }

    @Override
    public void start() {
        ;
    }

    public void loop() {
        telemetry.update();
        shooter.setHood(MANUALHOOD);
        shooter.updateBatteryVoltage();
        double shit = shooter.calculate();
        shooter.motorLeft.setPower(shit);
        shooter.motorRight.setPower(shit);
        if(gamepad1.right_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
            shooter.hoodPreset(constants.HOOD.MANUAL);
        }
        if(gamepad1.left_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.OFF);
            shooter.hoodPreset(constants.HOOD.RESET);
        }
        telemetry.update();
    }
}
