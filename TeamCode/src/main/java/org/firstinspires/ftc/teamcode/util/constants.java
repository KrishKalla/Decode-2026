package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class constants {

    //General Constants
    public static double ROBOT_MASS = 15; //kg
    public static double FORWARD_OFFSET = 5;
    public static double LATERAL_OFFSET = 0.5;

    //Intake Constants
    public static boolean REVERSED = false;
    public static double INTAKE_POWER = 0.8;
    public enum INTAKE {
        ON,
        OFF,
        REJECT
    }

    public enum INTAKE_EXTENSION {
        RETRACTED(0, 1),
        EXTENDED (1, 0);

        public final double left;
        public final double right;

        INTAKE_EXTENSION(double l, double r) {
            this.left = l;
            this.right = r;
        }
    }



}
