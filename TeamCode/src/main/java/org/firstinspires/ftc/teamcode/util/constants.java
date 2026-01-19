package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class constants {

    //General Constants
    public static double ROBOT_MASS = 15; //kg
    public static double FORWARD_OFFSET = 5; //in
    public static double LATERAL_OFFSET = 0.5; //in
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double LIMELIGHT_HEIGHT = 10; //in
    public static double LIMELIGHT_MOUNT_ANGLE = 45 * (Math.PI/180); //deg --> rad
    public static double APRIL_TAG_HEIGHT = 38.75 - 9.25; //in
    public static double APRIL_TAG_WIDTH = 6.5; //in
    public static double TICKS_PER_REV = 8192;

    //Intake Constants
    public static class intake {
        public static boolean REVERSED = false;
        public static double INTAKE_POWER = 0.8;
    }

    public enum INTAKE {
        ON,
        OFF,
        REJECT
    }

    public enum INTAKE_EXTENSION {
        RETRACTED(0, 1),
        EXTENDED(1, 0);

        public final double left;
        public final double right;

        INTAKE_EXTENSION(double l, double r) {
            this.left = l;
            this.right = r;
        }
    }


    //Turret Constants
    public static class turret {
        public static double SERVO_DEG_RANGE = 355; // needs to be reprogrammed to 270
        public static double GEAR_MULTIPLIER = 4.0 / 3.0;
        public static double SERVO_SPAN = 270;
        public static boolean IS_USING_ENCODER = false;
        public static double kP = 0.9;
        public static double deadband = 0.3;
        public static double step = 0.05;
    }

    public enum TURRET {
        RESET,
        MANUAL,
        AUTO
    }


    //Shooter Constants
    public static class shooter {
       public static double kP = 0.00080;
       public static double kI = 0;
       public static double kD = 0.00002;
       public static double kF = 0.00020;
       public static double TARGET_RPM = 5000;
       public static double alpha = 0.225;
       public static double step = 0.05;
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
