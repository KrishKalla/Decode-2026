package org.firstinspires.ftc.teamcode.OpModes.testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;

@TeleOp(name = "SRS Testing", group = "testing")
public class srstesting extends OpMode {
    SRSHub srs;
    SRSHub.Config config = new SRSHub.Config();

    @Override
    public void init() {
        config.setEncoder(1, SRSHub.Encoder.PWM);
        srs = hardwareMap.get(SRSHub.class, "srs");
        srs.init(config);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        srs.update();
        telemetry.addData("Encoder Absolute Pos", srs.readEncoder(1).position);
        telemetry.addData("disconnected:", srs.disconnected());
        telemetry.update();
    }
}
