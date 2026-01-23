package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake test", group = "Op_Tests")
@Config
public class intakeTesting extends LinearOpMode {

    public static double power = 0.0;

    @Override
    public void runOpMode() {

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftIntake");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightIntake");
        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()){

            leftShooter.setPower(power);
            rightShooter.setPower(power);
        }
    }
}
