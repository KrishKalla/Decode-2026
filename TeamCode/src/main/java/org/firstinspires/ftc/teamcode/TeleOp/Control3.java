package org.firstinspires.ftc.teamcode.TeleOp;// package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Controller", group = "Maggie")
public class Control3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {
        // Map hardware names from the Robot Controller configuration
        // Declare motor objects
        //edited **
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        /*
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
        } catch (Exception e) {
            telemetry.addData("Error", "config?");
            telemetry.update();
            return; // Stop if mapping fails
        }*/

        // Reverse one motor so both move forward with positive power //what does this mean copilot?
        /*leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);*/

        // Set zero power behavior to brake for better control
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //comment
        // Wait for the game to start
        waitForStart();

        // Run until the end of the match
        while (opModeIsActive()) {
            // Read joystick values (gamepad1 left stick Y, right stick Y)
            double forwardPower = gamepad1.left_stick_y * gamepad1.left_stick_y;  // Negative because up is -1
            // double rightPower = -gamepad1.right_stick_y;
            double backwardPower = -gamepad1.left_stick_y * gamepad1.left_stick_y;

            double posLposR = gamepad1.left_stick_x * gamepad1.left_stick_x;
            double nLnR = -gamepad1.left_stick_x * gamepad1.left_stick_x; //L

            double rotationPow = 1 - gamepad1.right_stick_x * gamepad1.right_stick_x;
            // Clip values to be safe (-1 to 1)
            // leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
            // rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            // Set motor power
            //forward
            if(gamepad1.left_stick_y > 0) {
                frontLeftDrive.setPower(forwardPower);
                frontRightDrive.setPower(forwardPower);
                backLeftDrive.setPower(forwardPower);
                backRightDrive.setPower(forwardPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }
            if(gamepad1.left_stick_y < 0) {
                frontLeftDrive.setPower(backwardPower);
                frontRightDrive.setPower(backwardPower);
                backLeftDrive.setPower(backwardPower);
                backRightDrive.setPower(backwardPower);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }


            //strafe
            if(gamepad1.left_stick_x < 0) {
                frontLeftDrive.setPower(posLposR);
                frontRightDrive.setPower(nLnR);
                backLeftDrive.setPower(nLnR);
                backRightDrive.setPower(posLposR);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }

            if(gamepad1.left_stick_x > 0) {
                frontLeftDrive.setPower(nLnR);
                frontRightDrive.setPower(posLposR);
                backLeftDrive.setPower(posLposR);
                backRightDrive.setPower(nLnR);
            /*leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);*/
            }

            if(gamepad1.right_stick_x > 0) {
                frontLeftDrive.setPower(forwardPower);
                frontRightDrive.setPower(rotationPow);
                backLeftDrive.setPower(forwardPower);
                backRightDrive.setPower(rotationPow);
            }

            if(gamepad1.right_stick_x < 0) {
                frontLeftDrive.setPower(rotationPow);
                frontRightDrive.setPower(forwardPower);
                backLeftDrive.setPower(rotationPow);
                backRightDrive.setPower(forwardPower);
            }

            // Send telemetry to Driver Station
            telemetry.addData("Forward power: ", forwardPower);
            telemetry.addData("Backward power: ", backwardPower);
            telemetry.addData("strafe Negative motor: ", nLnR);
            telemetry.addData("strafe Pos Motor: ", posLposR);

            telemetry.addData("LX ", gamepad1.left_stick_x);
            telemetry.addData("LY: ", gamepad1.left_stick_y);
            // telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}