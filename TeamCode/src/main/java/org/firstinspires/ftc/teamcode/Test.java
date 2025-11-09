package org.firstinspires.ftc.teamcode;

public class Test {

    private DcMotor test_motor;

    public void init(hardwareMap hwMap) {

        //reference configs (test_motor
        test_motor = hwMap.get(DcMotor.class, "test_motor");
        test_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //run using -- both will run at same speed (higher prob
        // run using --counter

    }

    public void setMotorSpeed(double speed) {
        //value -1-1
        test_motor.setPower(speed);
    }
}
