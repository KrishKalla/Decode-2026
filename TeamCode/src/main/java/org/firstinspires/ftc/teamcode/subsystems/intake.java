package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.constants;

public class intake {

    private DcMotorEx motorL;
    private DcMotorEx motorR;
    public Servo servoL;
    public Servo servoR;

    private DcMotorEx.Direction defaultL;
    private DcMotorEx.Direction defaultR;

    private String intakeState;
    private boolean extended;

    public intake(){
        intakeState = "INSTANTIATED";
        extended = false;
    }

    public void init(HardwareMap map) {
        intakeState = "INIT";

        motorL = map.get(DcMotorEx.class, "intakeL");
        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorR = map.get(DcMotorEx.class, "intakeR");
        motorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        servoL = map.get(Servo.class, "extensionL");
        servoR = map.get(Servo.class, "extensionR");

        if (constants.REVERSED) {
            motorL.setDirection(DcMotorEx.Direction.REVERSE);
            servoL.setDirection(Servo.Direction.REVERSE);
        } else {
            motorR.setDirection(DcMotorEx.Direction.REVERSE);
            servoR.setDirection(Servo.Direction.REVERSE);
        }

        defaultL = motorL.getDirection();
        defaultR = motorR.getDirection();
    }

    public void setIntake(constants.INTAKE state) {
        switch(state) {
            case ON:
                intakeState = "ON";
                setDirection(1);
                setPower(constants.INTAKE_POWER);
                setExtension(constants.INTAKE_EXTENSION.EXTENDED);
                break;
            case OFF:
                intakeState = "OFF";
                setDirection(1);
                setPower(0);
                setExtension(constants.INTAKE_EXTENSION.RETRACTED);
                break;
            case REJECT:
                intakeState = "REJECT";
                setDirection(-1);
                setPower(constants.INTAKE_POWER);
                setExtension(constants.INTAKE_EXTENSION.RETRACTED);
                break;
        }
    }

    public void setExtension(constants.INTAKE_EXTENSION state) {
        switch(state) {
            case RETRACTED:
                extended = false;
                servoL.setPosition(state.left);
                servoR.setPosition(state.right);
                break;
            case EXTENDED:
                extended = true;
                servoL.setPosition(state.left);
                servoR.setPosition(state.right);
                break;
        }
    }


    private void setDirection(int direction) {
        if (direction == -1) {
            motorL.setDirection(flip(defaultL));
            motorR.setDirection(flip(defaultR));
        }

        else {
            motorL.setDirection(defaultL);
            motorR.setDirection(defaultR);
        }
    }

    private void setPower(double d) {
        motorL.setPower(d);
        motorR.setPower(d);
    }

    private DcMotorEx.Direction flip(DcMotorEx.Direction dir) {
        return (dir == DcMotorEx.Direction.FORWARD)
                ? DcMotorEx.Direction.REVERSE
                : DcMotorEx.Direction.FORWARD;
    }

    public String getIntakeState() {
        return intakeState;
    }

    public boolean isExtended() {
        return extended;
    }
}
