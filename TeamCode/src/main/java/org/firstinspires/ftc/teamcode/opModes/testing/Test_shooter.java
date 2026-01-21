package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Test", group = "Op_Tests")
@Config
public class Test_shooter extends LinearOpMode {

    public static double power = 0.0;

    @Override
    public void runOpMode() {

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            leftShooter.setPower(power);
            rightShooter.setPower(power);

//            double tpsLeft  = leftShooter.getVelocity();
//            double tpsRight = rightShooter.getVelocity();
//
//            double rpmLeft  = (tpsLeft  / constants.TICKS_PER_REV) * 60.0;
//            double rpmRight = (tpsRight / constants.TICKS_PER_REV) * 60.0;
//            double rpmAvg   = (rpmLeft + rpmRight) / 2.0;
//
//            telemetry.addData("Set Power", power);
//
//            telemetry.addLine("=== Encoder Data ===");
//            telemetry.addData("Left RPM", rpmLeft);
//            telemetry.addData("Right RPM", rpmRight);
//            telemetry.addData("Avg RPM", rpmAvg);
//
//            telemetry.update();
        }
    }
}
