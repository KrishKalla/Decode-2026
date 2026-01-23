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
import org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "turret testing")
@Config
public class Test_turret extends OpMode {
    private turret turret;
    private LLHandler llhandler;
    private int alliance;
    public void init() {
        turret = new turret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        turret.init(hardwareMap, llhandler);
    }

    @Override
    public void start() {
        ;
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
            turret.preset(constants.TURRET.RESET);
        }
        telemetry.addLine(turret.toString());
        telemetry.addData("Calculated Target: ", turret.getCalculatedTarget());
        telemetry.addData("True position", turret.truePos);
        telemetry.update();
    }
}
