package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.utility.constants;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class spindex {
    private String state;
    public Servo spindexServo;
    private Servo blueServo;

    public int spinindex;

    public spindex() {state = "PRESET";}
// drivetrain bottomLeft and topLeft
    public void init(HardwareMap map) {
        spindexServo = map.get(Servo.class, "Spindex");
        blueServo = map.get(Servo.class, "Transfer");
        reset();
    }

    public void preset(constants.SPINDEX preset) {
        switch (preset) {
            case SPIN:
                spindexServo.setPosition(0.333/2*spinindex);
                spinindex++;
                if(spinindex == 3) {
                    spinindex = 0;
                }
                break;
            case PUSH:
                spindexServo.setPosition(constants.transfer + spinindex*0.333/2);
                spinindex++;
                if(spinindex == 3) {
                    spinindex = 0;
                }
                blueServo.setPosition(blueServo.getPosition() + constants.blueangle);
                // shake shit
                spindexServo.setPosition(spindexServo.getPosition() - constants.offset);
                for(int i = 0; i < (constants.shake - 1); i++) {
                    if((i % 2) == 0) {
                        spindexServo.setPosition(spindexServo.getPosition() + 2 * constants.offset);
                    }
                    else {
                        spindexServo.setPosition(spindexServo.getPosition() - 2 * constants.offset);
                    }
                }

                if((constants.shake - 1) % 2 == 0) {
                    spindexServo.setPosition(spindexServo.getPosition() - constants.offset);
                }
                if((constants.shake - 1) % 2 == 1) {
                    spindexServo.setPosition(spindexServo.getPosition() + constants.offset);
                }

                blueServo.setPosition(blueServo.getPosition() - 2* constants.blueangle);
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
}