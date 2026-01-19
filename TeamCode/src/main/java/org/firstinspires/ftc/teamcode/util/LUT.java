package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LUT {
    public static double D1 = 0.0,  A1 = 0.0;
    public static double D2 = 0.1,  A2 = 0.025;
    public static double D3 = 0.2,  A3 = 0.05;
    public static double D4 = 0.3,  A4 = 0.075;
    public static double D5 = 0.4,  A5 = 0.1;
    public static double D6 = 0.5,  A6 = 0.125;
    public static double D7 = 0.6,  A7 = 0.15;
    public static double D8 = 0.7,  A8 = 0.175;
    public static double D9 = 0.8,  A9 = 0.2;
    public static double D10 = 0.9, A10 = 0.225;

    public static double D11 = 1.0, A11 = 0.25;
    public static double D12 = 1.1, A12 = 0.275;
    public static double D13 = 1.2, A13 = 0.3;
    public static double D14 = 1.3, A14 = 0.325;
    public static double D15 = 1.4, A15 = 0.35;
    public static double D16 = 1.5, A16 = 0.375;
    public static double D17 = 1.6, A17 = 0.4;
    public static double D18 = 1.7, A18 = 0.425;
    public static double D19 = 1.8, A19 = 0.45;
    public static double D20 = 1.9, A20 = 0.475;

    public static double D21 = 2.0, A21 = 0.5;
    public static double D22 = 2.1, A22 = 0.525;
    public static double D23 = 2.2, A23 = 0.55;
    public static double D24 = 2.3, A24 = 0.575;
    public static double D25 = 2.4, A25 = 0.6;
    public static double D26 = 2.5, A26 = 0.625;
    public static double D27 = 2.6, A27 = 0.65;
    public static double D28 = 2.7, A28 = 0.675;
    public static double D29 = 2.8, A29 = 0.7;
    public static double D30 = 2.9, A30 = 0.725;

    public static double D31 = 3.0, A31 = 0.75;
    public static double D32 = 3.1, A32 = 0.775;
    public static double D33 = 3.2, A33 = 0.8;
    public static double D34 = 3.3, A34 = 0.825;
    public static double D35 = 3.4, A35 = 0.85;
    public static double D36 = 3.5, A36 = 0.875;
    public static double D37 = 3.6, A37 = 0.9;
    public static double D38 = 3.7, A38 = 0.925;
    public static double D39 = 3.8, A39 = 0.95;
    public static double D40 = 3.9, A40 = 0.975;

    public static double D41 = 4.0, A41 = 1.0;


    public static double get(double d) {
        double[] D = {
                D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,
                D11,D12,D13,D14,D15,D16,D17,D18,D19,D20,
                D21,D22,D23,D24,D25,D26,D27,D28,D29,D30,
                D31,D32,D33,D34,D35,D36,D37,D38,D39,D40,
                D41
        };
        double[] A = {
                A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,
                A11,A12,A13,A14,A15,A16,A17,A18,A19,A20,
                A21,A22,A23,A24,A25,A26,A27,A28,A29,A30,
                A31,A32,A33,A34,A35,A36,A37,A38,A39,A40,
                A41
        };

        if (d <= D[0]) return A[0];
        if (d >= D[D.length - 1]) return A[A.length-1];

        for (int i = 0; i <D.length - 1; i++) {
            double d0 = D[i], d1 = D[i+1];
            if (d >= d0 && d <= d1) {
                double a0 = A[i], a1 = A[i+1];

                if (d1 == d0) return a1;
                double t = (d-d0) / (d1-d0);
                return a0 + t*(a1-a0);
            }
        }
        return A[A.length - 1];
    }
}
