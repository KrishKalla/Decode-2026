package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp
public class PS extends OpMode {
    DcMotor frontLeft, backLeft, frontRight, backRight;
    @Override
    public void init() {
        frontLeft = HardwareMap.dcMotor.get("frontLeft");
        backLeft = HardwareMap.dcMotor.get("backLeft");
        frontRight = HardwareMap.dcMotor.get("frontRight");
        backRight = HardwareMap.dcMotor.get("backRight");
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
