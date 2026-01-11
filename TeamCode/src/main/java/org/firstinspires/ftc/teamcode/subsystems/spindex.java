package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.utility.constants.standard;

import org.firstinspires.ftc.teamcode.utility.constants;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class spindex {
    private String state;
    public Servo spindexServo;
    private Servo blueServo;
    public int spinindex = 1;
    private ElapsedTime timer;
    public spindex() {state = "PRESET";}
// drivetrain bottomLeft and topLeft
    public void init(HardwareMap map) {
        spindexServo = map.get(Servo.class, "Spindex");
        blueServo = map.get(Servo.class, "Transfer");
//        blueServo.setPosition(1);
        timer = new ElapsedTime();
        timer.reset();
        reset();
    }

    private double c = 0.135;

    public void preset(constants.SPINDEX preset) {
        switch (preset) {
            case SPIN:

                blueServo.setPosition(1);
                break;
            case PUSH:
                blueServo.setPosition(blueServo.getPosition() - constants.blueangle);
                break;
            case RESET:
                reset();
                break;
        }
    }

    @Override
    @NonNull
    public String toString() {
        return "SPINDEX POS: " + String.valueOf(spindexServo.getPosition());
    }

    private void reset() {
        state = "RESET";
        spindexServo.setPosition(0);
        state = "OFF";
    }

    public void spinRight() {
        spindexServo.setPosition(spindexServo.getPosition() + 0.01);
    }
    public void spinLeft() {
        spindexServo.setPosition(spindexServo.getPosition() - 0.01);
    }
}