package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.constants;

import java.util.Objects;

public class intake {

    private static final double BALL_DEBOUNCE_MS = 600;
    private DcMotorEx motorL;
    private DcMotorEx motorR;
    public Servo servoL;
    public Servo servoR;
    private AnalogInput breakbeamL;
    private AnalogInput breakbeamR;

    private DcMotorEx.Direction defaultL;
    private DcMotorEx.Direction defaultR;

    private String intakeState;
    private boolean extended;
    private boolean transfer_stalled = false;
    private boolean intake_stalled = false;
    private boolean blocked = false;
    private double emaL = 0;
    private double emaR = 0;

    private ElapsedTime stallTimer;



    private boolean spikeActive = false;
    private ElapsedTime spikeTimer = new ElapsedTime();


    private double transfer_reduction=0.7;

    private static final long INTAKE_STALL_DEBOUNCE_MS = 500; // ignore first 0.5s spike
    private ElapsedTime intake_stallTimer = new ElapsedTime();


    // ===== Stall Detection =====
    private boolean stallActive = false;
    private ElapsedTime stallDetectTimer = new ElapsedTime();

    private boolean firstspike=false;

    public intake(){

    }

    public void init(HardwareMap map) {

        intakeState = "INIT";


        motorL = map.get(DcMotorEx.class, "intakeL");
        motorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorR = map.get(DcMotorEx.class, "intakeR");
        motorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        servoL = map.get(Servo.class, "extensionL");
        servoR = map.get(Servo.class, "extensionR");
        servoR.setDirection(Servo.Direction.REVERSE);

        if (constants.intake.REVERSED) {
            motorL.setDirection(DcMotorEx.Direction.REVERSE);
            servoL.setDirection(Servo.Direction.REVERSE);
        } else {
            motorR.setDirection(DcMotorEx.Direction.REVERSE);
            servoR.setDirection(Servo.Direction.REVERSE);
        }

        defaultL = motorL.getDirection();
        defaultR = motorR.getDirection();


        breakbeamL = map.get(AnalogInput.class, "breakbeamL");
        breakbeamR = map.get(AnalogInput.class, "breakbeamR");

        extended = false;
    }

    public void update() {
        updateStallDetection();
        updateBreakbeams();
        if (transfer_stalled && intakeState.equals("ON")) {
            transfer_reduction=1;
        }
        if(Objects.equals(intakeState, "TRANSFERRING")){
            transfer_reduction=0.7;
            transfer_stalled=false;
        }

        if (Objects.equals(intakeState, "REJECT")) {
            transfer_stalled = false;
        }
    }
    private void updateStallDetection() {

        double current = getTransferCurrent();

        if (current > constants.intake.STALL_CURRENT_THRESHOLD  && intakeState.equals("ON")) {
            transfer_stalled=true;
        }
    }

    private void updateBreakbeams() {
        double L = breakbeamL.getVoltage();
        double R = breakbeamR.getVoltage();

        if (emaL == 0) {
            emaL = L;
        } else {
            emaL = constants.intake.alpha * L + (1-constants.intake.alpha) * emaL;
        }

        if (emaR == 0) {
            emaR = R;
        } else {
            emaR = constants.intake.alpha * R + (1-constants.intake.alpha) * emaR;
        }

        boolean rawBlocked = (emaL < constants.intake.breakbeamThreshold) || (emaR < constants.intake.breakbeamThreshold);

        if (rawBlocked) {
            if (!blocked) {
                blocked = true;
            }
        } else  {
            if (blocked) {
                blocked = false;
            }
        }

        if (blocked && transfer_stalled && Objects.equals(intakeState, "ON")) {
            setIntake(constants.INTAKE_PRESETS.OFF);
        }
    }

    public void setIntake(constants.INTAKE_PRESETS state) {
        switch(state) {
            case ON:
                intakeState = "ON";
                if (!blocked) {
                    setDirection(1);
                    setPowerR(constants.intake.INTAKE_POWER);
                    setPowerL(constants.intake.INTAKE_POWER - transfer_reduction);
                    setExtension(constants.INTAKE_EXTENSION.EXTENDED);
                }
                break;
            case OFF:
                intakeState = "OFF";
                setDirection(1);
                setPowerR(0);
                setPowerL(0);
                setExtension(constants.INTAKE_EXTENSION.RETRACTED);
                break;
            case REJECT:
                intakeState = "REJECT";
                setDirection(-1);
                setPowerR(constants.intake.INTAKE_POWER);
                setPowerL(constants.intake.INTAKE_POWER);
                setExtension(constants.INTAKE_EXTENSION.RETRACTED);
                break;
            case TRANSFERING:
                intakeState = "TRANSFERRING";
                setDirection(1);
                setPowerL(constants.intake.TRANSFER_POWER);
                setPowerR(constants.intake.TRANSFER_POWER);
                setExtension(constants.INTAKE_EXTENSION.RETRACTED);
                break;
            case GATE:
                intakeState = "GATE";
                setDirection(1);
                setPowerR(0);
                setPowerL(0);
                setExtension(constants.INTAKE_EXTENSION.GATE);
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
            case GATE:
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

    private void setPowerR(double d) {
        motorR.setPower(d);
    }

    private void setPowerL(double d) {
        motorL.setPower(d);
    }

    private DcMotorEx.Direction flip(DcMotorEx.Direction dir) {
        return (dir == DcMotorEx.Direction.FORWARD)
                ? DcMotorEx.Direction.REVERSE
                : DcMotorEx.Direction.FORWARD;
    }

    public String toString() {
        return "STATE: " + intakeState + "\n" +
                "POWER L: " + motorL.getPower() + "\n" +
                "POWER R: " + motorR.getPower() + "\n" +
                "INTAKE CURRENT: " + getIntakeCurrent() + "\n" +
                "TRANSFER CURRENT: " + getTransferCurrent() + "\n" +
                "BALL COUNT: " + constants.intake.ballCount + "\n" +
                "TRANSFER STALLED: " + transfer_stalled + "\n" +
                "INTAKE STALLED: " + intake_stalled;
    }

    public double getIntakeCurrent() {
        return motorR.getCurrent(CurrentUnit.AMPS);
    }
    public double getTransferCurrent() {
        return motorL.getCurrent(CurrentUnit.AMPS);
    }

    public String getIntakeState() {
        return intakeState;
    }

    public boolean isExtended() {
        return extended;
    }

    public boolean isBlocked() {
        return blocked;
    }

    public String getEMAs() {
        return emaL + ", " + emaR;
    }

    public String getRaw() {
        return breakbeamL.getVoltage() + ", " + breakbeamR.getVoltage();
    }
}
