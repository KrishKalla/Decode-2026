package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "turret testing")
@Config
public class turrettesting extends OpMode {
    private turret turret;
    private LLHandler llhandler;
    private int alliance = 1;
    public void init() {
        turret = new turret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        turret.init(hardwareMap, llhandler);
    }

    @Override
    public void start() {
        llhandler.start();
    }

    public void loop() {

        turret.update();
        if(turret.state.equals("AUTO")) {
            turret.update();
        }
        if(gamepad1.right_bumper) {
            turret.state = "AUTO";
        }
        if(gamepad1.left_bumper) {
            turret.state = "RESET";
            turret.preset(constants.TURRET_PRESETS.RESET);
        }
        telemetry.addLine(turret.toString());
        telemetry.addData("Calculated Target: ", turret.getCalculatedTarget());
//        telemetry.addData("True position", turret.truePos);
        telemetry.addData("LLResult", llhandler.getResult());
        telemetry.addData("handler results", Arrays.toString(llhandler.getLatestResult()));
        telemetry.update();
    }
}
