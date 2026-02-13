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
    public static boolean down = false;
    public static boolean on = false;

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
        if(down) {
            intake.setExtension(constants.INTAKE_EXTENSION.EXTENDED);
        }
        if(!down) {
            intake.setExtension(constants.INTAKE_EXTENSION.RETRACTED);
        }
        if (on) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        if (!on) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }
        telemetry.addLine(intake.toString());
        telemetry.update();
    }
}
//676767 - Han