package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Right Servo Hood", group = "Op_Tests")
@Config
public class Test_right_servo extends LinearOpMode {

    public static double position = 0.5;

    @Override
    public void runOpMode() {

        Servo leftHood = hardwareMap.get(Servo.class, "rightHood");

        waitForStart();

        while (opModeIsActive()) {
            leftHood.setPosition(position);

        }
    }
}
