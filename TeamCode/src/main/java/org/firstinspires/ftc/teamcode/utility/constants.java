package org.firstinspires.ftc.teamcode.utility;

public class constants {
    public static double transfer = 0.083;
    public static double offset = 0.03;
    public static double blueangle = 0.25;

    public static double modulationConstant = 0.05;

    public static int shake = 10;

    public enum INTAKE {
        TAKEIN,
        REJECT,
        OFF,
        RESET
    }

    public enum SHOOTER {
        SHOOTFAR,
        SHOOTSHORT,
        OFF,
        RESET
    }

    public enum SPINDEX {
        SPIN,
        PUSH,
        RESET
    }
}
