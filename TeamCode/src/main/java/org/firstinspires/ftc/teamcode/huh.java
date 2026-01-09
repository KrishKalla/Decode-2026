package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class huh extends LinearOpMode {
    DcMotor funky=null;
    DcMotor topright=null;
    DcMotor bottomleft=null;
    DcMotor bottomright=null;
    public void runOpMode(){
        funky=hardwareMap.get(DcMotor.class,"funky");
        telemetry.addData("Does this work?","please work");
        funky.setPower(4);

    }
}
