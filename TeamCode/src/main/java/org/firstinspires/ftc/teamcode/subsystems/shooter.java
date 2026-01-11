package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.utility.constants;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utility.constants;

public class shooter {
    private String state;
    private DcMotor motor;

    public shooter() {state = "PRESET";}

    public void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "Shooter");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        reset();
    }

    public void preset(constants.SHOOTER preset) {
        switch (preset) {
            case SHOOTFAR:
                state = "SHOOTFAR";
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                motor.setPower(0.67); // tune manually
                break;
            case SHOOTSHORT:
                state = "SHOOTSHORT";
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                motor.setPower(0.51); // tune manually
                break;
            case OFF:
                state = "OFF";
                motor.setPower(0.0);
                break;
            case REVERSE:
                state= "REVERSE";
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setPower(0.2);
                break;
            case RESET:
                reset();
                break;
        }
    }

    public void modulate(int direction) {
        motor.setPower(motor.getPower() + direction * constants.modulationConstant);
    }
    @Override
    @NonNull
    public String toString() {
        return "MOTOR POWER: " + String.valueOf(motor.getPower());
    }

    private void reset() {
        state = "RESET";
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
        state = "OFF";
    }
}