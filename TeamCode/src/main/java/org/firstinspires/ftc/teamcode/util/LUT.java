package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LUT {
    static double[][] table = {
            {0.83, 650, 0.24},
            {0,99, 650, 0.24},
            {1.03, 660, 0,285},
            {1.06, 670, 0.34},
            {1.09, 680, 0.375},
            {1.13, 690, 0.4},
            {1.17, 690, 0.475},
            {1.2, 700, 0.5}

    };
    // change ts to a regression algo
    public static double[] get(double d) {
        // weighted LUT
        double rpm = 0.0;
        double hood = 0.0;
        for(int i = 0; i < 7; i++) {
            if(table[i][0] < d && d < table[i+1][0]) {
                double interval = table[i+1][0] - table[i][0];
                double weight1 = (d - table[i][0])/interval;
                double weight2 = (table[i+1][0] - d)/interval;
                rpm = weight1 * table[i][1] + weight2 * table[i+1][1];
                hood = weight1 * table[i][2] + weight2 * table[i+1][2];
                break;
            }
        }
        return new double[] {rpm, hood};
    }
}
