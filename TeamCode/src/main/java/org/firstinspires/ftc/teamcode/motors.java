package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class motors {

    private DcMotor test_motor;
    private double ticksPerRev;
    //depends on config

    public void init(HardwareMap hwMap) {
        //insert touch sens

        test_motor = hwMap.get(DcMotor.class, "test_motor");
        test_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev=test_motor.getMotorType().getTicksPerRev();
        test_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorSpeed(double speed) {
        test_motor.setPower(speed);
    }

}
