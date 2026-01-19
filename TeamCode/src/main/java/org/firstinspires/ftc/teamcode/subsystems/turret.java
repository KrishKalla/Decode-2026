package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

// control algorithm: tx -> 0
public class turret {
    private Servo left;
    private Servo right;
    private LLHandler handler;
    private DcMotorEx encoder;

    private double truePos;
    private double calculatedTarget = 0.0;
    private double previousError = 0.0;

    private String state;

    public turret() {}

    public void init(HardwareMap map, LLHandler handler) {
        left = map.get(Servo.class, "turretLeft");
        right = map.get(Servo.class, "turretRight");
        this.handler = handler;
        if(constants.turret.IS_USING_ENCODER) {
            encoder = map.get(DcMotorEx.class, "turretEncoder");
        }
        state = "INIT";
    }

    public void preset(constants.TURRET preset) {
        switch (preset) {
            case RESET:
                setServoPos(0.5);
                calculatedTarget = 180.0;
                state = "START";
                break;
            case AUTO:
                break;
            case MANUAL:
                state = "MANUAL";
                break;
        }
    }

    public double update() {
        this.previousError = handler.getLatestResult()[3];
        if (previousError == -1001) {
            state = "MISSING";
            previousError = 0;
        }
        else {
            state = "TRACKING";
            if (Math.abs(previousError) < constants.turret.deadband) {
                previousError = 0;
            }

            if (constants.turret.IS_USING_ENCODER) {
                truePos = (encoder.getCurrentPosition() / constants.TICKS_PER_REV) * 360;
                calculatedTarget = truePos + constants.turret.kP * previousError;
                calculatedTarget = ((calculatedTarget % 360) + 360) % 360;
                return (calculatedTarget / constants.turret.GEAR_MULTIPLIER) / constants.turret.SERVO_DEG_RANGE;
            }
        }

        calculatedTarget += constants.turret.kP * previousError;
        calculatedTarget = ((calculatedTarget %360)+360)%360;
        double pos = (calculatedTarget /constants.turret.GEAR_MULTIPLIER)/constants.turret.SERVO_DEG_RANGE;
        setServoPos(pos);
        return pos;
    }

    private void setServoPos(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    // @param direction -- expects -1, 1 where -1 is left & 1 is right
    public void manual(int direction) {
        left.setPosition(Math.max(0, Math.min(1, left.getPosition() + direction * constants.turret.step)));
        right.setPosition(Math.max(0, Math.min(1, right.getPosition() + direction * constants.turret.step)));
    }

    @NonNull
    @Override
    public String toString() {
        return "Turret State: " + state + "\n" +
                "Calculated Target: " + calculatedTarget + "\n";
    }
}
