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
    public static int mode=0;

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
        if (mode==1) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        else if (mode==2) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }
        else if (mode==3){
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
        }
        else if (mode==4){
            intake.setIntake(constants.INTAKE_PRESETS.GATE);
        }
        telemetry.addLine(intake.toString());
        telemetry.update();
    }
}