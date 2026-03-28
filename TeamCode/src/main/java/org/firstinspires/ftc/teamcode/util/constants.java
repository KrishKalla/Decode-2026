package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {

    //General Constants
    public static double ROBOT_MASS = 11.97; //kg
    public static double FORWARD_OFFSET = 0; //in
    public static double LATERAL_OFFSET = 0; //in
    public static double NOMINAL_VOLTAGE = 13.5;
    public static double LIMELIGHT_HEIGHT = 0.41; //deg
    public static double LIMELIGHT_MOUNT_ANGLE = 17.45128; //deg --> rad
    public static double APRIL_TAG_HEIGHT = 38.75 - 9.25; //in
    public static double APRIL_TAG_WIDTH = 6.5; //in
    public static double TICKS_PER_REV = 8192;
    public static double STALL_CURRENT = 2;

    //Intake Constants
    @Config
    public static class intake {
        public static boolean REVERSED = false;
        public static double INTAKE_POWER = 1;
        public static double TRANSFER_POWER = 1;
        public static int ballCount=0;
        public static double STALL_CURRENT_THRESHOLD = 2.1;
    }

    public enum INTAKE_PRESETS {
        ON,
        OFF,
        REJECT,
        GATE,
        TRANSFERING
    }

    @Config
    public enum INTAKE_EXTENSION {

        RETRACTED(0.37, 0.37),
        GATE(0.72,0.72),
        EXTENDED(0.66, 0.66);

        public final double left;
        public final double right;

        INTAKE_EXTENSION(double l, double r) {
            this.left = l;
            this.right = r;
        }
    }

    //Shooter Constants
    @Config
    public static class shooter {
       public static double kP = 0.008;
       public static double kS = 0.0042;
       public static double kV = 0.00042;
       public static double TARGET_RPM = 800;
       public static double alpha = 0.225;
       public static double step = 0.02;
       public static double MIN_ANGLE = 0.18;
       public static double PASSTHROUGH = 0.85;
       public static double STOP = 0.475;
       public static double SHOT_LOAD = 65;
       public static double Target_Hood = 0.18;
       public static double Hood_pos = 0.18;
       public static double Hood_delta =0.01;
    }

    public enum HOOD {

        RESET,
        MANUAL,
        AUTO
    }

    public enum FLYWHEEL {
        OFF,
        ON
    }

    // need to be tunedddd

    public static double rpmToleranceLow = 25;
    public static double rpmToleranceHigh = 60;

    public static double driveScaleNormal = 1.0;
    public static double driveScaleMedium = 0.75;
    public static double driveScaleHeavy = 0.55;

    public static double movingRpmGain = 6.0;

}
