package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.utility.constants.SHOOTER.SHOOT;
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
        motor = map.get(DcMotor.class, "shooterMotor");
        reset();
    }

    public void preset(constants.SHOOTER preset) {
        switch (preset) {
            case SHOOTFAR:
                state = "SHOOTFAR";
                motor.setPower(0.67);
                break;
            case SHOOTSHORT:
                state = "SHOOTSHORT";
                motor.setPower(0.41);
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