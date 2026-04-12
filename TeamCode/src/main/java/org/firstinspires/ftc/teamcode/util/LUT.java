package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LUT {
    static double[][] table = {
            {.93, 1260, .25},
            {1.10, 1300, .291},
            {1.23, 1340, .345},
            {1.37, 1370, .395},
            {1.46, 1370, .455},
            {1.51, 1370, .467},
            {1.57, 1380, .4775},
            {1.67, 1400, .4815},
            {1.71, 1420, .5075},
            {1.83, 1460, .535},
            {1.99, 1480, .555},
            {2.11, 1510, .57},
            {2.28, 1530, .585},
            {2.45, 1560, .60},
//            {1.51, 1450, 0.65},
//            {1.35, 1420, 0.6},
//            {1.13, 1380, 0.53},
//            {1.77, 1490, 0.69},
//            {1.89, 1550, 0.735},
//            {2.01, 1590, 0.76},
//            {2.18, 1640, 0.8},
//            {2.35,1680,0.855},
//            {2.51, 1720, 0.895},
            {2.71, 1600, 0.6},
            {3.15, 1770, 0.59},
            {3.35, 1820, 0.62},
            {3.45, 1860, 0.69},
            {3.67, 1920, 0.80}
    };

    public static double[] get(double d) {
        // Handle out of bounds - below minimum distance
        if(d <= table[0][0]) {
            return new double[] {table[0][1], table[0][2]};
        }

        // Handle out of bounds - above maximum distance
        if(d >= table[table.length-1][0]) {
            return new double[] {table[table.length-1][1], table[table.length-1][2]};
        }

        // Weighted LUT interpolation
        double rpm = 0.0;
        double hood = 0.0;
        for(int i = 0; i < table.length - 1; i++) {
            if(table[i][0] <= d && d <= table[i+1][0]) {
                double interval = table[i+1][0] - table[i][0];
                double weight1 = (table[i+1][0] - d) / interval;  // Weight for table[i]
                double weight2 = (d - table[i][0]) / interval;     // Weight for table[i+1]
                rpm = weight1 * table[i][1] + weight2 * table[i+1][1];
                hood = weight1 * table[i][2] + weight2 * table[i+1][2];
                break;
            }
        }
        return new double[] {rpm, hood};
    }
}