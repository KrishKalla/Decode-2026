package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class singleIntakeTesting extends OpMode {
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "intakeL");
        motor2 = hardwareMap.get(DcMotorEx.class, "intakeR");
    }

    public void loop() {
        motor1.setPower(0.6);

        motor1.setPower(0);
        motor2.setPower(-0.6);

        motor2.setPower(0);
    }
}
