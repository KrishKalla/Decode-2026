package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Gamepad extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop() {
        double speedForward = -gamepad1.left_stick_y;
        telemetry.addData("x", gamepad1.left_stick_x);
        //when click dot, easily find out of list
        //gamepad . = a, b, qfewugsyd
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("a button", gamepad1.a);

        //up and left is negative, right and down oppo
        //w/speedfwd it's reversed

        //if build now, values for x,y (left stick) and 'a' button visible

        //it is gamepad 1 if controller is turned on clicking 'a
        //it is gamepad 2 if controller is turned on clicking 'b

    }
}
