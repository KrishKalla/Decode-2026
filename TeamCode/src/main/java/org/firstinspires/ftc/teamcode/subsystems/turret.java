package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LLHandler;

@Config
public class Turret {
    private FtcDashboard dashboard;

    //Hardware
    private Servo left;
    private Servo right;
    private DcMotor encoder;
    private LLHandler llhandler;
    private Follower follower;

    private int alliance = 1;

    //Constants
    public static double SERVO_TO_TURRET_RATIO = 1.33;
    private static final double ENCODER_TO_TURRET_RATIO = 108/21.0;
    private static final double ENCODER_TICKS_PER_REV = 8192.0;
    private static final double MIN_ANGLE = -135;
    private static final double MAX_ANGLE = 135.0;
    private static final double TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) * ENCODER_TO_TURRET_RATIO;
    public static double TURRET_OFFSET = -2.7266;
    public static double ENCODER_DIRECTION = -1;
    public static double LLWEIGHT = 0.75;

    private double target;
    private boolean aimed;
    private boolean llValid;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double TOLERANCE = 0.5;

    public static double SNAP = 12.0;
    public static double minServoStep = 0.003;
    public static double maxServoStep = 0.015;
    public static double iClamp = 40.0;
    private double iTerm = 0.0;
    private double lastErr = 0.0;

    private ElapsedTime timer;

    public Turret() {

    }

    public void init(HardwareMap map, LLHandler handler, Follower follower) {
        left = map.get(Servo.class, "turretLeft");
        left.setDirection(Servo.Direction.REVERSE);
        right = map.get(Servo.class, "turretRight");
        right.setDirection(Servo.Direction.REVERSE);

        encoder = map.get(DcMotorEx.class, "intakeL"); //PORT 2 EXPANSKON HUB
        llhandler = handler;

        this.follower = follower;

        dashboard = FtcDashboard.getInstance();

        timer = new ElapsedTime();
    }

    public void zeroTurret() {
        target = 0;
        left.setPosition(0.5);
        right.setPosition(0.5);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        timer.reset();
    }

    public void update(Pose goal) {
        setTargetAngle(calculateAngleToGoal(goal));

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) {
            dt = 0.02;
        }

        double current = getCurrentAngle();
        double errDeg = normalizeAngle(target - current);

        double snapServo = clamp(angleToServoPosition(target), 0.11, 0.89);

        if (Math.abs(errDeg) > SNAP) {
            left.setPosition(snapServo);
            right.setPosition(snapServo);
            iTerm = 0.0;
            lastErr = errDeg;
            aimed = false;
        }

        iTerm += errDeg * dt;
        iTerm = clamp(iTerm, -iClamp, iClamp);

        double dErr = (errDeg - lastErr) / dt;
        lastErr = errDeg;

        double uDeg = kP * errDeg + kI * iTerm + kD * dErr;

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
        double nextServo = clamp(currentServo + servoDelta, 0.11, 0.89);

        boolean saturatingHigh = nextServo >= 0.89 && servoDelta > 0;
        boolean saturatingLow = nextServo <= 0.11 && servoDelta < 0;
        if (saturatingHigh || saturatingLow) {
            iTerm = -errDeg * dt;
            iTerm = clamp(iTerm, -iClamp, iClamp);
        }

        left.setPosition(nextServo);
        right.setPosition(nextServo);

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
        return encoder.getCurrentPosition() / TICKS_PER_TURRET_DEGREE * ENCODER_DIRECTION;
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

    public void setTargetAngle(double angle) {
        target = angle;
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
