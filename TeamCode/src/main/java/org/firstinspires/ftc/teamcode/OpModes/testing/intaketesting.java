package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.util.constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "intake testing")
@Config
public class intaketesting extends OpMode {
    private intake intake;

    public void init() {
        intake = new intake();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake.init(hardwareMap);
    }

    @Override
    public void start() {
        ;
    }

    public void loop() {
        if(gamepad1.right_bumper) {
            intake.setIntake(constants.INTAKE.ON);
        }
        if(gamepad1.left_bumper) {
            intake.setIntake(constants.INTAKE.REJECT);
        }
        if(gamepad1.triangle) {
            intake.setIntake(constants.INTAKE.OFF);
        }
        telemetry.addLine(intake.toString());
        telemetry.update();
    }
}
