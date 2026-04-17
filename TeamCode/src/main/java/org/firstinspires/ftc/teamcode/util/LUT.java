package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LUT {
    static double[][] table = {
            {.93, 1260, .25},
            {1.10, 1300, .291},
            {1.23, 1340, .345},
            {1.37, 1370, .395},
            {1.46, 1390, .66},
            {1.51, 1400, .6675},
            //Bullet
            {1.56, 1430, .675},
            {1.68, 1450, .685},
            {1.77, 1480, .695},
            {1.86, 1510, .7},
            {1.96, 1550, .715},
            {2.09, 1560, .7275},
            {2.20, 1560, .735},
            //Lob
            {2.28, 1530, .585},
            {2.45, 1560, .60},


            {2.71, 1770, 0.6},
            //Far
//            {3.15, 1770, 0.59},
//            {3.35, 1820, 0.62},
//            {3.45, 1860, 0.69},
//            {3.67, 1920, 0.80},
            {3.23, 1820, 0.915},
            {3.36, 1850, 0.92},
            {3.55, 1880, 0.925},
            {3.68, 1920, 0.92},
            {3.88, 1980, 0.915}
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