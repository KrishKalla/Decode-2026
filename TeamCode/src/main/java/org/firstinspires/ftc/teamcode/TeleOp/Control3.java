package org.firstinspires.ftc.teamcode.TeleOp;// package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Controller", group = "Maggie")
public class Control3 extends LinearOpMode {


    @Override
    public void runOpMode() {
        // Map hardware names from the Robot Controller configuration
        // Declare motor objects
        //edited **
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;

        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
        } catch (Exception e) {
            telemetry.addData("Error", "config?");
            telemetry.update();
            return; // Stop if mapping fails
        }

        // Reverse one motor so both move forward with positive power //what does this mean copilot?
        /*leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);*/

        // Set zero power behavior to brake for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        // Run until the end of the match
        while (opModeIsActive()) {
            // Read joystick values (gamepad1 left stick Y, right stick Y)
            double forwardPower = gamepad1.left_stick_y * gamepad1.left_stick_y;  // Negative because up is -1
            // double rightPower = -gamepad1.right_stick_y;
            double backwardPower = -gamepad1.left_stick_y * gamepad1.left_stick_y;

            double strafeRPower = gamepad1.left_stick_x * gamepad1.left_stick_x;
            double strafeLPower = -gamepad1.left_stick_x * gamepad1.left_stick_x;
            // Clip values to be safe (-1 to 1)
            // leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
            // rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            // Set motor power
            //forward
            if(gamepad1.left_stick_y > 0) {
                frontLeft.setPower(forwardPower);
                frontRight.setPower(forwardPower);
                backLeft.setPower(forwardPower);
                backRight.setPower(forwardPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }
            if(gamepad1.left_stick_y < 0) {
                frontLeft.setPower(backwardPower);
                frontRight.setPower(backwardPower);
                backLeft.setPower(backwardPower);
                backRight.setPower(backwardPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }


            //strafe
            if(gamepad1.left_stick_x < 0) {
                frontLeft.setPower(strafeLPower);
                frontRight.setPower(strafeRPower);
                backLeft.setPower(strafeRPower);
                backRight.setPower(strafeLPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }

            if(gamepad1.left_stick_x > 0) {
                frontLeft.setPower(strafeRPower);
                frontRight.setPower(strafeLPower);
                backLeft.setPower(strafeLPower);
                backRight.setPower(strafeRPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }

            // Send telemetry to Driver Station
            telemetry.addData("Forward power: ", forwardPower);
            telemetry.addData("Backward power: ", backwardPower);
            telemetry.addData("StrafeL: ", strafeLPower);
            telemetry.addData("StrafeR: ", strafeRPower);

            telemetry.addData("LX ", gamepad1.left_stick_x);
            telemetry.addData("LY: ", gamepad1.left_stick_y);
            // telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}