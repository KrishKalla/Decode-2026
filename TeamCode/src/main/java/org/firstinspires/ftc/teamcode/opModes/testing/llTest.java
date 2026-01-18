package org.firstinspires.ftc.teamcode.opModes.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.LLHandler;


@TeleOp(name = "LimeLight Testing")
public class llTest extends OpMode {
    LLHandler llhandler;

    @Override
    public void init() {
        llhandler = new LLHandler(hardwareMap, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //would be used in Regular TeleOp
    @Override
    public void init_loop() {
        if (gamepad1.circle) {
            llhandler.alliance(0);
        }
        else if(gamepad1.square) {
            llhandler.alliance(1);
        }
    }

    @Override
    public void start() {
        llhandler.start();
    }

    @Override
    public void loop() {
        telemetry.addData("x: ", llhandler.poll()[0]);
        telemetry.addData("y: ", llhandler.poll()[1]);
        telemetry.addData("z: ", llhandler.poll()[2]);
        telemetry.addData("handler: ", llhandler.toString());
        telemetry.update();
    }
}
