package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class Controls extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    public void runOpMode() {
        try {

            /*change configs on driver hub;
            front left = funky1
            front right = funky2
            back left = funky3
            back right = funky4
            */

            motorFL = hardwareMap.get(DcMotor.class, "funky1");
            motorFR = hardwareMap.get(DcMotor.class, "funky2");
            motorBL = hardwareMap.get(DcMotor.class, "funky3");
            motorBR = hardwareMap.get(DcMotor.class, "funky4");

            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.FORWARD);

            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("Motor initialized.");
            telemetry.update();
            waitForStart();

        while (opModeIsActive()) {
            // Example: Run motor with gamepad control
            double powerF = -gamepad1.left_stick_y; // Forward/backward on left stick
            double powerS = gamepad1.left_stick_x;

            //ie 1, 0.7 SF= 0.3 ~~ 1, -0.7 SF3= 0.3
            //ie 1, -0.7 SF2= 0.3 ~~ 1, 0.7 SF4= 0.3
            double botSF = 1-powerS;
            double botSF2 = 1+powerS;
            //(unnecessary)
            double botSF3 = 1-powerS;
            double botSF4 = 1+powerS;

            motorFL.setPower(powerF);
            motorFR.setPower(powerF);
            motorBL.setPower(powerF);
            motorBR.setPower(powerF);

            //fwd right
            if (gamepad1.left_stick_x>0 && gamepad1.left_stick_y>0) {
                //bench.setMotorSpeed(0.5);
                motorFR.setPower(botSF);
                motorBR.setPower(botSF);
            }
            else {
                motorFR.setPower(powerF);
                motorBR.setPower(powerF);
            }
            //fwd left
            if (gamepad1.left_stick_x<0 && gamepad1.left_stick_y>0) {
                //bench.setMotorSpeed(0.5);
                motorFL.setPower(botSF2);
                motorBL.setPower(botSF2);
            }
            else {
                motorFL.setPower(powerF);
                motorBL.setPower(powerF);
            }


            //bkw
            //left back
            if (gamepad1.left_stick_x>0 && gamepad1.left_stick_y<0) {
                //bench.setMotorSpeed(0.5);
                motorFR.setPower(botSF3);
                motorBR.setPower(botSF3);
            }
            else {
                motorFR.setPower(powerF);
                motorBR.setPower(powerF);
            }

            //right back
            if (gamepad1.left_stick_x< 0 && gamepad1.left_stick_y< 0) {
                //bench.setMotorSpeed(0.5);
                motorFL.setPower(botSF4);
                motorBL.setPower(botSF4);
            }
            else {
                motorFL.setPower(powerF);
                motorBL.setPower(powerF);
            }

            // Send telemetry to Driver hub
            telemetry.addData("Motor FWD", powerF);
            telemetry.addData("Motor SDE", powerS);

            telemetry.addData("Right", botSF);
            telemetry.addData("Left", botSF2);

            telemetry.update();

        }

        // Stop motor when OpMode ends
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    } catch (Exception e) {
        telemetry.addData("Error", e.getMessage());
        telemetry.update();
        }
    }
}