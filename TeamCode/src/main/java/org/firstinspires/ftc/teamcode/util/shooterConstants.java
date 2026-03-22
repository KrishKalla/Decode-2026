package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

@Config
public class shooterConstants {
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 26; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //rad
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static double HOOD_MIN_ANGLE = 19.173; //deg
    public static double MIN_HOOD = 0.18;
    public static double HOOD_MAX_ANGLE = 51.815; //deg
    public static double MAX_HOOD = 1;
    public static double SERVO_RANGE = 355; //deg

    public static double FLYWHEEL_MIN_SPEED = 500;
    public static double FLYWHEEL_MAX_SPEED = 1500;


    public static double getFlywheelTicksFromVelocity(double velocity) {
        return MathFunctions.clamp( 94.501 * velocity / 12 - 187.96, FLYWHEEL_MIN_SPEED,
                FLYWHEEL_MAX_SPEED) ;
    }

    public double getHoodTicksFromDegrees(double deg) {
        return 0.025121 * deg - 0.301645;
    }
}
