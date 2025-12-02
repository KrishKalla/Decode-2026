package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class UsingMotors extends OpMode {

    //si quieres conectar a robot, toca el buton desde "TeamCode"
    //luego lo construye (build)

    //make sure control hub is connected to computer either
    //via usb or wifi

    motors bench = new motors();

    @Override
    public void init(){
        bench.init(hardwareMap);
        //init config
    }
    @Override
    public void loop(){
        bench.setMotorSpeed(0.5);
    }
}
