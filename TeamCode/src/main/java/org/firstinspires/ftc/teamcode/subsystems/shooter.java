package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.LUT;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.shooterConstants;

public class shooter {
    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    private Servo left;
    private Servo right;
    private Servo stopper;
    private LLHandler handler;
    private VoltageSensor battery;
    private PIDFController pidf;
    private ElapsedTime timer;
    private PoseTracker poseTracker;

    private double previousDistance;
    private double ema = -101;
    private double power = 0;
    private double rpm;

    private double voltage = 12.0;
    private double maxVoltageCompensation = 1.25;
    private double minVoltageCompensation = 0.90;

    public String hoodState;
    private String hoodTrackingState;
    private boolean flywheelState;
    private boolean stopped = true;

    public shooter() {

    }

    public void init(HardwareMap map, LLHandler handler) {
        motorLeft = map.get(DcMotorEx.class, "leftShooter");
        motorRight = map.get(DcMotorEx.class, "rightShooter");
        motorRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left = map.get(Servo.class, "leftHood");
        right = map.get(Servo.class, "rightHood");
        this.handler = handler;

        stopper = map.get(Servo.class, "stopper");

        timer = new ElapsedTime();
        timer.reset();

        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : map.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0 && v < minVoltage) {
                minVoltage = v;
                battery = vs;
            }
        }
        voltage = (battery != null) ? battery.getVoltage() : 12.0;

        hoodState = "INIT";
        flywheelState = false;
    }

    public void init(HardwareMap map, LLHandler handler, PoseTracker dt) {
        poseTracker = dt;
        init(map, handler);
    }

    public void hoodPreset(constants.HOOD preset) {
        switch (preset) {
            case RESET:
                setHood(constants.shooter.MIN_ANGLE);
                hoodState = "RESET";
                break;
            case MANUAL:
                hoodState = "MANUAL";
                break;
            case AUTO:
                hoodState = "AUTO";
                break;
        }
    }

    public void flywheelPreset(constants.FLYWHEEL preset) {
        switch (preset) {
            case OFF:
                power = 0;
                motorLeft.setPower(power);
                motorRight.setPower(power);
                flywheelState = false;
                break;
            case ON:
                flywheelState = true;
                break;
        }
    }

    public void update() {
        previousDistance = handler.getLatestResult()[2];
        if (previousDistance == -1001) {
            hoodTrackingState = "MISSING";
        } else {
            hoodTrackingState = "TRACKING";
            filterDistance(previousDistance);
            double hoodPos = LUT.get(ema);
            setHood(hoodPos);
        }

        updateBatteryVoltage();

        if (flywheelState) {
            power = calculate();
            motorLeft.setPower(power);
            motorRight.setPower(power);
        } else {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }

    public double calculate() {

        double rpmL = motorLeft.getVelocity();
        double rpmR = motorRight.getVelocity();

        rpm = (rpmL + rpmR) / 2;

        double setpoint = constants.shooter.TARGET_RPM;
        double error = setpoint - rpm;

        double feedforward = constants.shooter.kS + constants.shooter.kV * setpoint;
        double feedback = constants.shooter.kP * error;

        double power = feedforward + feedback;

        double scale = constants.NOMINAL_VOLTAGE / voltage;
        scale = Math.max(minVoltageCompensation, Math.min(maxVoltageCompensation, scale));
        power *= scale;

        power = Math.max(-1.0, Math.min(1.0, power));
        return power;
    }

    public void updateBatteryVoltage () {
        if (battery == null) return;

        if (timer.seconds() >= 0.15) {
            double v = battery.getVoltage();
            if (v > 0) {
                voltage = v;
            }
            timer.reset();
        }
    }

    private void filterDistance ( double raw){
        if (ema == -101) {
            ema = raw;
        } else {
            ema = ema + constants.shooter.alpha * (raw - ema);
        }
    }

    public void setHood ( double pos){
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void manual(double direction) {
        left.setPosition(Math.max(0, Math.min(1, left.getPosition() + direction * constants.shooter.step)));
        right.setPosition(Math.max(0, Math.min(1, right.getPosition() + direction * constants.shooter.step)));
    }

    public void setStopper(boolean isStopped) {
        if (!isStopped) {
            stopper.setPosition(constants.shooter.PASSTHROUGH);
            stopped = false;
        } else {
            stopper.setPosition(constants.shooter.STOP);
            stopped = true;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return  "Flywheel State: " + flywheelState + "\n" +
                "Hood State: " + hoodState + "\n" +
                "Hood Tracking State: " + hoodTrackingState +
                "Stopper: " + stopped + "\n" +
                "EMA: " + ema + "\n" +
                "Hood Position: " + left.getPosition() + "\n" +
                "RPM: " + rpm + "\n" +
                "Power: " + power + "\n" +
                "Voltage: " + voltage;
    }

    public double getPower() {
        return power;
    }

    public double getRPM() {
        return ((motorLeft.getVelocity() + motorRight.getVelocity())/2);
    }

    public double mToIn(double meters) {
        return meters * 39.3700787;
    }

}
