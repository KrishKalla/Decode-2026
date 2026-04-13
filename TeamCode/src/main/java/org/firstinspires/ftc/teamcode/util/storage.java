package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class storage {
    public static Pose lastBlueAutoPose = new Pose(115.23, 125.38, Math.toRadians(180));
    public static Pose lastRedAutoPose = new Pose(115.75, 125.38,Math.toRadians(0));

    public static double BLUE_X = 6.7;
    public static double BLUE_Y = 140;

    public static double RED_X = 137;
    public static double RED_Y = 140;

    public static int counter = 0;
}
