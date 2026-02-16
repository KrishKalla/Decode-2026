package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.storage;

@Config
public class Turret {
    private FtcDashboard dashboard;

    //Hardware
    private Servo left;
    private Servo right;
    private DcMotor encoder;
    private Follower follower;

    //Constants
    public static double SERVO_TO_TURRET_RATIO = 1.3333333;
    private static final double ENCODER_TO_TURRET_RATIO = 108/21.0;
    private static final double ENCODER_TICKS_PER_REV = 8192.0;
    private static final double MIN_ANGLE = -135;
    private static final double MAX_ANGLE = 135.0;
    private static final double TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) * ENCODER_TO_TURRET_RATIO;
    public static double TURRET_OFFSET = -2.7266;
    public static double ENCODER_DIRECTION = -1;
    public static double SERVO_MAX = 0.80;
    public static double SERVO_MIN = 0.20;
    public int zero = 0;

    private double target;
    private double lastTarget;
    private boolean aimed;

    public static double kP = 0.115;
    public static double kI = 0;
    public static double kD = 0.002;
    public static double kF = 0.00165;
    public static double TOLERANCE = 0.5;

    public static double SNAP = 180;
    public static double minServoStep = 0.00025;
    public static double maxServoStep = 0.4;
    public static double iClamp = 40.0;
    private double iTerm = 0.0;
    private double lastErr = 0.0;

    private ElapsedTime timer;

    public Turret() {

    }

    public void init(HardwareMap map, Follower follower) {
        left = map.get(Servo.class, "turretLeft");
        left.setDirection(Servo.Direction.REVERSE);
        right = map.get(Servo.class, "turretRight");
        right.setDirection(Servo.Direction.REVERSE);

        encoder = map.get(DcMotorEx.class, "intakeL"); //PORT 2 EXPANSION HUB

        this.follower = follower;

        dashboard = FtcDashboard.getInstance();

        timer = new ElapsedTime();
    }

    public void reset() {
        setTargetAngle(0);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zero = 0;
        timer.reset();
    }

    public void zeroTurret() {
        zero = storage.storedZero;
    }

    public void TEST_RESET_ONLY () {
        left.setPosition(0.5);
        right.setPosition(0.5);
    }

    public void update(double a) {
        setTargetAngle(a);
        updatePID();
    }

    public void update(Pose goal) {
        if (goal != null) {
            setTargetAngle(normalizeAngle(calculateAngleToGoal(goal)));
        }
        updatePID();
    }

    private void updatePID() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) {
                dt = 0.02;
        }

        double current = getCurrentAngle();
        double errDeg = (target - current);

        double snapServo = clamp(angleToServoPosition(target), 0.145, 0.855);

        if (Math.abs(errDeg) > SNAP) {
            left.setPosition(snapServo);
            right.setPosition(snapServo);
            iTerm = 0.0;
            lastErr = errDeg;
            aimed = false;
            return;
        }

        iTerm += errDeg * dt;
        iTerm = clamp(iTerm, -iClamp, iClamp);

        double dErr = (errDeg - lastErr) / dt;
        lastErr = errDeg;

        double targetRate = normalizeAngle(target - lastTarget) / dt;
        lastTarget = target;

        double uDeg = kP * errDeg + kI * iTerm + kD * dErr + kF * targetRate;

        double servoDelta = (uDeg / SERVO_TO_TURRET_RATIO / 355);

        if (Math.abs(errDeg) > TOLERANCE) {
            servoDelta = Math.copySign(
                    Math.max(Math.abs(servoDelta), minServoStep),
                    servoDelta);
        } else {
            servoDelta = 0.0;
        }

        servoDelta = clamp(servoDelta, -maxServoStep, maxServoStep);

        double currentServo = left.getPosition();

        double nextServo = clamp(currentServo + servoDelta, SERVO_MIN, SERVO_MAX);

        boolean saturatingHigh = nextServo >= SERVO_MAX && servoDelta > 0;
        boolean saturatingLow = nextServo <= SERVO_MIN && servoDelta < 0;
        if (saturatingHigh || saturatingLow) {
            iTerm -= errDeg * dt;
            iTerm = clamp(iTerm, -iClamp, iClamp);
        }

        left.setPosition(nextServo);
        right.setPosition(nextServo);

        storage.storedZero = zero-encoder.getCurrentPosition();

        aimed = Math.abs(errDeg) <= TOLERANCE;
    }

    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double getCurrentAngle() {
        return getDelta() / TICKS_PER_TURRET_DEGREE * ENCODER_DIRECTION;
    }

    public Pose getTurretFieldPose() {
        Pose robotPose = follower.getPose();

        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double worldOffsetX = TURRET_OFFSET * cosH;
        double worldOffsetY = TURRET_OFFSET * sinH;

        double turretX = robotPose.getX() + worldOffsetX;
        double turretY = robotPose.getY() + worldOffsetY;

        return new Pose(turretX, turretY, heading);
    }

    private double calculateAngleToGoal(Pose goal) {
        Pose turretPose = getTurretFieldPose();

        double dX = goal.getX() - turretPose.getX();
        double dY = goal.getY() - turretPose.getY();

        double fieldAngleToGoal = Math.atan2(dY, dX);

        return normalizeAngle(Math.toDegrees(fieldAngleToGoal - turretPose.getHeading() + Math.PI));
    }

    private void setTargetAngle(double angle) {
        target = angle;
    }

    public void setManualAngle(double angle) {
        left.setPosition(angleToServoPosition(angle));
        right.setPosition(angleToServoPosition(angle));
    }

    private double angleToServoPosition(double angle) {
        return 0.5 + (angle / SERVO_TO_TURRET_RATIO) / 355;
    }

    private double clamp(double pos, double low, double high) {
        return (Math.max(low, Math.min(high, pos)));
    }

    public void drawTurret(Canvas canvas, TelemetryPacket packet) {
        double turretAngle = Math.toRadians(getCurrentAngle());
        Pose turretPose = getTurretFieldPose();

        double turretX = turretPose.getX();
        double turretY = turretPose.getY();

        double turretHeading = turretPose.getHeading() + turretAngle;

        canvas.setStroke("#63BF00");
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(turretX, turretY, 3.44); //turret radius

        double startX = turretX + 1 * Math.cos(turretHeading);
        double startY = turretY + 1 * Math.sin(turretHeading);

        double endX = turretX + 3.44 * Math.cos(turretHeading);
        double endY = turretY + 3.44 * Math.sin(turretHeading);

        canvas.strokeLine(startX, startY, endX, endY);

        packet.put("Turret X", turretX);
        packet.put("Turret Y", turretY);
        packet.put("Turret Field Heading", Math.toDegrees(turretHeading));
        packet.put("Turret Absolute Heading", Math.toDegrees(turretAngle));
    }

    public int getDelta() {
        return encoder.getCurrentPosition() - zero;
    }

    public int getEncoderPos() {
        return encoder.getCurrentPosition();
    }

    public double getError() {
        return normalizeAngle(target - getCurrentAngle());
    }

    public double getTargetAngle() {
        return target;
    }

    public boolean isAimed() {
        return aimed;
    }
}