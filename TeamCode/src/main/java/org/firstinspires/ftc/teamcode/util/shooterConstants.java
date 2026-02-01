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

    public static double MIN_HOOD_DEG = 19.173; //deg
    public static double MIN_HOOD_ANGLE = 0.18;
    public static double MAX_HOOD_DEG = 51.815; //deg
    public static double MAX_HOOD = 1;
    public static double SERVO_RANGE = 355; //deg


//    public double getFlywheelTicksFromVelocity(double vel) {
//        return MathFunctions.clamp()
//    }
    public double getHoodTicksFromDegrees(double deg) {
        return 0.02696 * deg - 0.3969;
    }
}
