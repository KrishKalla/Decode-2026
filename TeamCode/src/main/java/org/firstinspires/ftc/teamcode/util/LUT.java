package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LUT {
    static double[][] table = {
            {1.14, 700, 0.43},
            {1.36, 720, 0.47},
            {1.58, 750, 0.55},
            {1.8, 770, 0.62},
            {2.09, 820, 0.67},
            {2.30, 850, 0.74},
            {2.5, 890, 0.79}
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