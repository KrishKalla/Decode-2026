package org.firstinspires.ftc.teamcode.subsystems.experimental;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.SRSHub;
import org.firstinspires.ftc.teamcode.util.storage;

public class turret {
    private FtcDashboard dashboard;

    public double SERVO_Pose =0;

    //Hardware
    private Servo left;
    private Servo right;
    private Follower follower;
    private SRSHub hub;
    private SRSHub.Config config = new SRSHub.Config();

    //Constants
    public static double SERVO_TO_TURRET_RATIO = 1.333333333;
    private static final double ENCODER_TO_TURRET_RATIO = 108/21.0;
    private static final int ENCODER_TICKS_PER_REV = 1024;
    private static final double MIN_ANGLE = -135;
    private static final double MAX_ANGLE = 135.0;
    private static final double TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) * ENCODER_TO_TURRET_RATIO;
    public static double TURRET_OFFSET = -2.7265858268;
    public static int ENCODER_DIRECTION = 1;
    public static int DIRECTION = 1;
    public static double SERVO_MAX = 0.85;
    public static double SERVO_MIN = 0.15;
    public static int ZERO = 423;
    public static double SLOP = 3;
    public static double TOLERANCE = 0.5;

    private double target;
    private int lastPos;
    private double pos;

    private boolean aimed;


    public void init(HardwareMap map, Follower follower) {
        left = map.get(Servo.class, "turretLeft");
        left.setDirection(Servo.Direction.REVERSE);
        right = map.get(Servo.class, "turretRight");
        right.setDirection(Servo.Direction.REVERSE);

        config.setEncoder(1, SRSHub.Encoder.PWM);
        hub = map.get(SRSHub.class, "srs");
        hub.init(config);

        this.follower = follower;

        dashboard = FtcDashboard.getInstance();

        lastPos = hub.readEncoder(1).position;
    }

    public void reset() {
        setTargetAngle(0);
    }

    public void zeroTurret() {
        setTargetAngle(0);
        storage.counter=0;
    }

    public void TEST_RESET_ONLY () {
        left.setPosition(0.5);
        right.setPosition(0.5);
        storage.counter = 0;
    }


    public void update(Pose goal) {
        if (goal != null) {
            setTargetAngle(normalizeAngle(calculateAngleToGoal(goal)));
        }
    }

    public void update(double a) {
        setTargetAngle(a);
    }

    public void periodic() {
        if (Math.abs(getError()) < TOLERANCE) {
            aimed = true;
        } else {
            aimed = false;
            if (Math.abs(getError()) < SLOP) {
                double servoDelta = angleToServoDelta(getError());
                double currentPos = pos;
                double targetPos = currentPos + servoDelta;
                targetPos = clamp(targetPos, SERVO_MIN, SERVO_MAX);
                hardwareUpdate(targetPos);
            } else {
                pos = angleToServoPosition(target);
                pos = clamp(pos, SERVO_MIN, SERVO_MAX);
                hardwareUpdate(pos);
            }
        }
    }

    public void hardwareUpdate(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
        SERVO_Pose=pos;
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

    private double calculateAngleToGoal(@NonNull Pose goal) {
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

    public double angleToServoPosition(double angle) {
        return 0.5 + (angle / SERVO_TO_TURRET_RATIO) / 355;
    }

    public double angleToServoDelta(double angle) {
        return (angle / SERVO_TO_TURRET_RATIO) / 355;
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
        packet.put("Servo Pos", SERVO_Pose);
    }

    public int getDelta() {
        hub.update();
        return getEncoderPos() + storage.counter*1024 - ZERO;
    }

    public int getEncoderPos() {
        int pos = hub.readEncoder(1).position;
        int d = pos - lastPos;
        int half = ENCODER_TICKS_PER_REV / 2;
        if (d < -half) {
            storage.counter += DIRECTION;
        } else if (d > half) {
            storage.counter -= DIRECTION;
        }
        lastPos = pos;
        return pos;
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
