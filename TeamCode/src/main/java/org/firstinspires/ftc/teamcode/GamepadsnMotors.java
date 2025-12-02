package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp
@Disabled
public class GamepadsnMotors extends OpMode {
    private DcMotor test_motor;
    private double ticksPerRev;
    motors bench = new motors();


    @Override
    public void init(){
        bench.init(hardwareMap);
        //init config
    }

    @Override
    public void loop(){
        double motor1Speed = -gamepad1.left_stick_y;
        //bench.setMotorSpeed(motor1Speed);

        if (gamepad1.left_stick_y>=0.3 || gamepad1.left_stick_y<=-0.3) {
            //bench.setMotorSpeed(0.5);
            test_motor.setPower(motor1Speed);
        }
        /*else if (gamepad1.left_stick_y<=-0.3) {
            //bench.setMotorSpeed(-0.5);
            test_motor.setPower(motor1Speed);
        }*/
        else {
            //bench.setMotorSpeed(0.0);
            test_motor.setPower(0.0);
        }
    }
}
