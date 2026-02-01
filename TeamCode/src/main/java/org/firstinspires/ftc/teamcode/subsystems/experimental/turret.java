package org.firstinspires.ftc.teamcode.subsystems.experimental;

import static java.lang.Math.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LLHandler;

@Config
public class turret {
    private Servo left;
    private Servo right;
    private LLHandler handler;
    private DcMotorEx encoder;
    private ElapsedTime timer;
    private PoseTracker poseTracker;

    private double alliance = 1;

    private static final double SERVO_TO_TURRET_RATIO  = 4.0 / 3.0;
    private static final double ENCODER_TO_TURRET_RATIO = 108.0 / 21.0;
    private static final double ENCODER_TICKS_PER_REV  = 8192.0;
    private static final double MIN_ANGLE = -135.0;
    private static final double MAX_ANGLE = 135.0;
    private static final double ENCODER_TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) / ENCODER_TO_TURRET_RATIO;
    private static final double TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) / ENCODER_TO_TURRET_RATIO;

    private int zeroTicks = 0;
    private double turretTargetDeg = 0.0;

    public turret() {}


    public void init(HardwareMap map, LLHandler handler, PoseTracker tracker) {
        left = map.get(Servo.class, "turretLeft");
        right = map.get(Servo.class, "turretRight");
        this.handler = handler;
        encoder = map.get(DcMotorEx.class, "turretEncoder");
        poseTracker = tracker;

        left.setPosition(0.5);
        right.setPosition(0.5);

        zeroTurret();

        timer = new ElapsedTime();
        timer.reset();
    }

    public void zeroTurret() {
        zeroTicks = encoder.getCurrentPosition();
        turretTargetDeg = 0.0;
        timer.reset();
    }


    public void update(Pose goalPose) {
        double angleToGoal = calculateTurretAngleToGoal(goalPose);
        double constrainedAngle = getConstrainedAngle(angleToGoal);
        setTurretAngle(constrainedAngle);
    }

    //Normalize between [-180 and 180]
    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double getCurrentTurretAngle() {
        int currentTicks = encoder.getCurrentPosition();
        int deltaTicks = currentTicks - zeroTicks;
        return deltaTicks/TICKS_PER_TURRET_DEGREE;
    }

    public double calculateTurretAngleToGoal(Pose goalPose) {
        Pose currentPose = poseTracker.getPose();

        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        double worldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(currentPose.getHeading());
        double robotFrameAngle = normalizeAngle(worldAngleToGoal - robotHeading);

        return normalizeAngle(robotFrameAngle + 180.0);
    }

    public double getConstrainedAngle(double desiredAngle) {
        double normalized = normalizeAngle(desiredAngle);

        if (normalized >= MIN_ANGLE && normalized <= MAX_ANGLE) {
            return normalized;
        }

        double distTo135 = Math.abs(normalizeAngle(MAX_ANGLE - normalized));
        double distToNeg135 = Math.abs(normalizeAngle(MIN_ANGLE - normalized));

        return (distTo135 <= distToNeg135) ? MAX_ANGLE : MIN_ANGLE;
    }

    public void setTurretAngle(double turretAngleDeg) {
        turretTargetDeg = turretAngleDeg;

        double servoAngleDeg = turretAngleDeg / SERVO_TO_TURRET_RATIO;
        double servoPosition = (servoAngleDeg / 355) + 0.5;

        servoPosition = clamp(servoPosition, 0.11, 0.87);

        left.setPosition(servoPosition);
        right.setPosition(servoPosition);
    }

    public boolean isAimedAtGoal(Pose goalPose, double toleranceDeg) {
        double currentAngle = getCurrentTurretAngle();
        double angleToGoal = calculateTurretAngleToGoal(goalPose);
        double constrainedGoal = getConstrainedAngle(angleToGoal);

        double error = Math.abs(normalizeAngle(constrainedGoal - currentAngle));
        return error <= toleranceDeg;
    }

    public double getError() {
        double currentAngle = getCurrentTurretAngle();
        return normalizeAngle(turretTargetDeg - currentAngle);
    }

}



