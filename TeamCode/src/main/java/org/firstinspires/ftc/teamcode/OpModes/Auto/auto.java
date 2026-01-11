package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class auto extends LinearOpMode {
    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    HardwareMap map = hardwareMap;
    double speed = 0.4;

    public void runOpMode() {
    FL =map.get(DcMotorEx .class,"topLeft");
            FL.setDirection(DcMotorEx.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    FR =map.get(DcMotorEx .class,"topRight");
    //FR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    BL =map.get(DcMotorEx .class,"bottomLeft");
            BL.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    BR =map.get(DcMotorEx .class,"bottomRight");
    //BR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    while (opModeInInit()) {

    }
    waitForStart();

    ElapsedTime timer = new ElapsedTime();
    timer.reset();
    while (timer.milliseconds() < 1000)
        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);

    }
}
