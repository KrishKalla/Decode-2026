package org.firstinspires.ftc.teamcode.subsystems;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.constants;

import java.util.Objects;
public class intake {
    private String state;
    private DcMotor motor;

    public intake() {state = "PRESET";}

    public void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "motorIntake");
        reset();
    }

    public void preset(constants.INTAKE preset) {
        switch (preset) {
            case TAKEIN:
                state = "TAKEIN";
                motor.setPower(1.0);
                break;
            case OFF:
                state = "OFF";
                motor.setPower(0.0);
                break;
            case RESET:
                reset();
                break;
        }
    }

    @Override
    @NonNull
    public String toString() {
        return "MOTOR POWER: " + String.valueOf(motor.getPower());
    }

    private void reset() {
        state = "RESET";
        motor.setPower(0);
        state = "OFF";
    }
}