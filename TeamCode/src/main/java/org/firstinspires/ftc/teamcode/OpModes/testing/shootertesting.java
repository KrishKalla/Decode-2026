package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Configurable
public class shootertesting extends OpMode {

    private boolean automatedDrive;
    private TelemetryManager telemetry;
    private shooter shooter = new shooter();

    private int alliance = 0;
    private LLHandler llhandler;

    public void init() {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
    }

    @Override
    public void start() {
        ;
    }

    public void loop() {
        telemetry.update();
        shooter.hoodPreset(constants.HOOD.MANUAL);
        shooter.setHood(1);
        shooter.updateBatteryVoltage();
        double shit = shooter.calculate();
        shooter.motorLeft.setPower(shit);
        shooter.motorRight.setPower(shit);
        if(gamepad1.right_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
        }
        if(gamepad1.left_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.OFF);
            shooter.hoodPreset(constants.HOOD.RESET);
        }

    }
}
