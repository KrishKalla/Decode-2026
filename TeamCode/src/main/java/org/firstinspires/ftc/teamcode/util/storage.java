package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class storage {
    public static int alliance = 0;
    public static Pose lastBlueAutoPose = new Pose(23.6, 127.213, Math.toRadians(145));
    public static Pose lastRedAutoPose = new Pose(120.179, 127.973,Math.toRadians(36));

    public static double BLUE_X = 10;
    public static double BLUE_Y = 140;

    public static double RED_X = 140;
    public static double RED_Y = 140;

    public static int storedZero;
}
