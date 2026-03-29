package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.localization.PoseTracker;
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

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

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

    public boolean auto = true;
    private double previousDistance;
    public double ema = -101;
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
                auto = false;
                break;
            case AUTO:
                hoodState = "AUTO";
                auto = true;
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



    public void calculateParams() {
        double dist = handler.getLatestResult()[2];
        if (dist == -1001) {
            hoodTrackingState = "MISSING";
        } else {
            previousDistance = Math.sqrt(Math.pow(dist, 2) - Math.pow(constants.APRIL_TAG_HEIGHT - constants.LIMELIGHT_HEIGHT, 2));
            hoodTrackingState = "TRACKING";
            filterDistance(previousDistance);
            double[] interp = LUT.get(ema);
            constants.shooter.Target_Hood= interp[1];
            constants.shooter.TARGET_RPM = interp[0];
        }
    }

    public void far(){
        constants.shooter.TARGET_RPM=940;
        constants.shooter.Target_Hood=0.6767;
    }

    public void update() {
        updateBatteryVoltage();

        if (flywheelState) {
            power = calculate();
            motorLeft.setPower(power);
            motorRight.setPower(power);
        } else {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
        setHood(constants.shooter.Hood_pos);
    }

    public double calculate() {

        double rpmL = -motorLeft.getVelocity();
        double rpmR = -motorRight.getVelocity();

        rpm = (rpmL + rpmR) / 2;

        double setpoint = constants.shooter.TARGET_RPM;
        double error = setpoint - rpm;

        constants.shooter.Hood_pos=constants.shooter.Target_Hood;
        double feedforward = constants.shooter.kS + constants.shooter.kV * setpoint;
        double feedback = constants.shooter.kP * error;

        double power = feedforward + feedback;

        double scale = constants.NOMINAL_VOLTAGE / voltage;
        scale = Math.max(minVoltageCompensation, Math.min(maxVoltageCompensation, scale));
        power *= scale;

        power = Math.max(0, Math.min(1.0, power));
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


//    public Vector calculateShotVectorAndUpdateTurret(double robotHeading) {
//        //constants
//        double g = 32.174 * 12;
//        double x = robotToGoalVector.getMagnitude() - shooterConstants.PASS_THROUGH_POINT_RADIUS;
//        double y = shooterConstants.SCORE_HEIGHT;
//        double a = shooterConstants.SCORE_ANGLE;
//        //calculate initial launch components
//        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math. tan(a)), shooterConstants.HOOD_MAX_ANGLE,
//                shooterConstants.HOOD_MIN_ANGLE);
//
//        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math. pow(Math.cos(hoodAngle), 2) * (x * Math.tan (hoodAngle) - y)));
//        //get robot velocity and convert it into parallel and perpendicular components
//        Vector robotVelocity = hardware.poseTracker.getVelocity();
//
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//
//        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        //velocity compensation variables
//        double vz = flywheelSpeed * Math.sin(hoodAngle);
//        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
//        double ndr = nvr * time;
//        //recalculate launch components
//        hoodAngle = MathFunctions.clamp (Math.atan(vz / nvr), shooterConstants. HOOD_MAX_ANGLE,
//                shooterConstants.HOOD_MIN_ANGLE);
//
//        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math. pow(Math. cos (hoodAngle), 2) * (ndr * Math. tan(hoodAngle) - y)));
//
//        //update turret
//        double turretVelComp0ffset = Math.atan(perpendicularComponent / ivr);
//        double turretAngle = Math. toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelComp0ffset);
//
//        if (turretAngle > 180) {
//            turretAngle -= 360;
//        }
//
//        constants.shooter.TARGET_RPM=shooterConstants.getFlywheelTicksFromVelocity(flywheelSpeed);
//    }



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
        return ((-(motorLeft.getVelocity() + motorRight.getVelocity())/2));
    }
    public double getHoodAngle() {
        return left.getPosition();
    }
    public double mToIn(double meters) {
        return meters * 39.3700787;
    }

}
