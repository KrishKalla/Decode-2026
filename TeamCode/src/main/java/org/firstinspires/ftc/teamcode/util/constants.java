package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {

    //General Constants
    public static double ROBOT_MASS = 10; //kg
    public static double FORWARD_OFFSET = 2.9331; //in
    public static double LATERAL_OFFSET = -6.24015; //in
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
    }

    public enum INTAKE_PRESETS {
        ON,
        OFF,
        REJECT,
        GATE, TRANSFERING
    }

    @Config
    public enum INTAKE_EXTENSION {

        RETRACTED(0.35, 0.37),
        EXTENDED(0.66, 0.68);

        public final double left;
        public final double right;

        INTAKE_EXTENSION(double l, double r) {
            this.left = l;
            this.right = r;
        }
    }


    //Turret Constants
    @Config
    public static class turret {
        public static double SERVO_DEG_RANGE = 355; // needs to be reprogrammed to 270
        public static double GEAR_MULTIPLIER = 4.0 / 3.0;
        public static double SERVO_SPAN = 270;
        public static boolean IS_USING_ENCODER = false;
        public static double kP = 0.9;
        public static double deadband = 0.3;
        public static double step = 0.05;
    }

    public enum TURRET_PRESETS {
        RESET,
        MANUAL,
        AUTO
    }


    //Shooter Constants
    @Config
    public static class shooter {
       public static double kP = 0.004;
       public static double kS = 0.0042;
       public static double kV = 0.00042;
       public static double TARGET_RPM = 800;
       public static double alpha = 0.225;
       public static double step = 0.02;
       public static double MIN_ANGLE = 0.18;
       public static double PASSTHROUGH = 0.9;
       public static double STOP = 0.55;
       public static double SHOT_LOAD = 65;
       public static double Hood_pos = 0.18;
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
}
