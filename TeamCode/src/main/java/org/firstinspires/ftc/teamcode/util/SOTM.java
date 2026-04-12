package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class SOTM {
    private static Pose goalPose = null;
    private static Pose adjustedGoal;
    public static double K = 0.002;

    public static void setGoalPose(Pose goalPose) {
        SOTM.goalPose = goalPose;
    }

    public static void calculate(double vx, double vy) {
        if (goalPose == null) {
            throw new IllegalStateException("setGoalPose() was not called [goalPose = null], thus SOTM cannot initialize");
        }

        adjustedGoal = new Pose(
                goalPose.getX() - K * vx,
                goalPose.getY() - K * vy
        );
    }
    public static Pose getAdjustedGoal() {
        return adjustedGoal;
    }

}
