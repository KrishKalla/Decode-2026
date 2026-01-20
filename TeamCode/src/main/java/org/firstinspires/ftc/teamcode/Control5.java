package org.firstinspires.ftc.teamcode;// package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Controller", group = "Maggie")
public class Control5 extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        telemetry.addData("motors", "calibrated :D");


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            axial = Math.copySign(axial * axial, axial); //chat gpt
            lateral = Math.copySign(lateral * lateral, lateral);
            yaw = Math.copySign(yaw * yaw, yaw);
            //the copy sign makes sure the square doesn't impact the -+ value


            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (max > 1.0) { //this code also max<-1.0
                frontLeftPower  /= max; //divides greatest xDirectionPower by itself to get max;1
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontRightDrive.setPower(frontRightPower);
            frontLeftDrive.setPower(frontLeftPower);
            backRightDrive.setPower(backRightPower);
            backLeftDrive.setPower(backLeftPower);



            telemetry.addData("Front Left ", frontLeftPower);
            telemetry.addData("Front Right ", frontRightPower);
            telemetry.addData("Back Left ", backLeftPower);
            telemetry.addData("Back Right ", backRightPower);


            telemetry.addData("LX ", gamepad1.left_stick_x);
            telemetry.addData("LY: ", gamepad1.left_stick_y);

            telemetry.update();
        }
    }
}
