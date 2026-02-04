package org.firstinspires.ftc.teamcode.subsystems.experimental;

import com.acmerobotics.dashboard.config.Config;
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
    private Servo left;
    private Servo right;
    private LLHandler handler;
    public DcMotorEx encoder;
    private ElapsedTime timer;
    private PoseTracker poseTracker;

    private double alliance = 1;

    private static final double SERVO_TO_TURRET_RATIO  = 4.0 / 3.0;
    private static final double ENCODER_TO_TURRET_RATIO = 108.0 / 21.0;
    private static final double ENCODER_TICKS_PER_REV  = 8192.0;
    private static final double MIN_ANGLE = -135.0;
    private static final double MAX_ANGLE = 135.0;
    private static final double TICKS_PER_TURRET_DEGREE =
            (ENCODER_TICKS_PER_REV / 360.0) * ENCODER_TO_TURRET_RATIO;

    private double turretTargetDeg = 0.0;
    private boolean llValid;
    public static double TURRET_OFFSET = -2.7266;
    public static double ENCODER_DIRECTION = 1.0;
    public static double ERROR_SIGN = 1;
    public static double LLWEIGHT = 1;

    public Turret() {}


    public void init(HardwareMap map, LLHandler handler, PoseTracker tracker) {
        left = map.get(Servo.class, "turretLeft");
        left.setDirection(Servo.Direction.REVERSE);
        right = map.get(Servo.class, "turretRight");
        right.setDirection(Servo.Direction.REVERSE);

        this.handler = handler;
        encoder = map.get(DcMotorEx.class, "frontLeft");
        poseTracker = tracker;

        timer = new ElapsedTime();
    }

    public void zeroTurret() {
        left.setPosition(0.5);
        right.setPosition(0.5);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretTargetDeg = 0.0;
        timer.reset();
    }


    public void update(Pose goalPose) {
        double angleToGoal = calculateTurretAngleToGoal(goalPose);
        double constrainedAngle = getConstrainedAngle(angleToGoal);
        setTurretAngle(constrainedAngle);
        double correctedAngle = applyCorrection(angleToGoal);
        double LLCorrectedAngle = applyLLCorrection(correctedAngle);
        constrainedAngle = getConstrainedAngle(LLCorrectedAngle);
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
        int deltaTicks = encoder.getCurrentPosition();
        return deltaTicks / TICKS_PER_TURRET_DEGREE * ENCODER_DIRECTION;
    }

    public Pose getTurretFieldPose() {
        Pose robotPose = poseTracker.getPose();

        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double worldOffsetX = TURRET_OFFSET * cosH;
        double worldOffsetY = TURRET_OFFSET * sinH;

        double turretX = robotPose.getX() + worldOffsetX;
        double turretY = robotPose.getY() + worldOffsetY;

        return new Pose(turretX, turretY, heading);
    }

    public double calculateTurretAngleToGoal(Pose goalPose) {
        Pose currentPose = getTurretFieldPose();

        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        double worldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(currentPose.getHeading() + Math.PI);
        double robotFrameAngle = normalizeAngle(worldAngleToGoal - robotHeading);

        return normalizeAngle(robotFrameAngle);
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
        turretTargetDeg = normalizeAngle(turretAngleDeg);

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

    private double applyLLCorrection(double calculatedAngle) {
        if (handler.getLatestResult() != null) {
            double headingOffset = handler.getLatestResult()[3];
            if (headingOffset == -1001) {
                llValid = false;
                return calculatedAngle;
            }
            llValid = true;
            return calculatedAngle + (headingOffset * LLWEIGHT);
        }
        llValid = false;
        return calculatedAngle;
    }

    private double applyCorrection(double calculatedAngle) {
        double error = getError();
        double correctedPos =  (calculatedAngle + ERROR_SIGN * error / SERVO_TO_TURRET_RATIO) / 355;
        correctedPos = clamp(correctedPos, 0.11, 0.87);
        return correctedPos;
    }

    public double getError() {
        double currentAngle = getCurrentTurretAngle();
        return normalizeAngle(turretTargetDeg - currentAngle);
    }

    public double getTurretTargetDeg() {
        return turretTargetDeg;
    }

    public boolean getLLValid() {
        return llValid;
    }

    public double clamp(double pos, double low, double high) {
        return (Math.max(low, Math.min(high, pos)));
    }

}



