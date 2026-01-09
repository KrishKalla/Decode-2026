package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Control2 extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop() {
        double speedFB = gamepad1.left_stick_y * gamepad1.left_stick_y; //forward or backward
        double strafeLR = gamepad1.left_stick_x * gamepad1.left_stick_x; //left or right

        Telemetry.addData("Rx", gamepad1.right_stick_x);
        Telemetry.addData("Ry", gamepad1.right_stick_y);

        Telemetry.addData("Lx", gamepad1.left_stick_x);
        Telemetry.addData("Ly", gamepad1.left_stick_y);

        Telemetry.addData("speedFB", speedFB);
        Telemetry.addData("strafe", strafeLR);
    }
}
