package org.firstinspires.ftc.teamcode.OpModes.partnerAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Partner Auto - Strafe Left", group = "Partner Auto")
public class blueFarPark extends LinearOpMode {

    // ── Tune these ────────────────────────────────────────────────
    static final double STRAFE_POWER  = 0.5;   // 0.0 – 1.0
    static final double STRAFE_TIME   = 0.5;   // seconds
    // ─────────────────────────────────────────────────────────────
    @Override
    public void runOpMode() {
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse right-side motors so positive power = forward on all
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Strafe LEFT:  FL=-, FR=+, BL=+, BR=-
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && (timer.seconds() < STRAFE_TIME)) {
            frontLeft.setPower(STRAFE_POWER);
            frontRight.setPower( -STRAFE_POWER);
            backLeft.setPower( -STRAFE_POWER);
            backRight.setPower(STRAFE_POWER);
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}