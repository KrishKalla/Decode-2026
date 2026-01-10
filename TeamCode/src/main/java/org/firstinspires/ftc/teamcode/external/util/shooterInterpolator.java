package org.firstinspires.ftc.teamcode.external.util;

public class shooterInterpolator {
    private final shooterLookup table;

    public shooterInterpolator(shooterLookup table) {
        this.table = table;
    }

    public double getAngle(double x, double y) {

        double spacing = table.getSpacing();
        double minX = table.getMinX();
        double minY = table.getMinY();

        double fx = (x - minX) / spacing;
        double fy = (y - minY) / spacing;

        // Clamp
        fx = Math.max(0, Math.min(fx, table.getNx() - 1));
        fy = Math.max(0, Math.min(fy, table.getNy() - 1));

        int ix0 = (int) Math.floor(fx);
        int iy0 = (int) Math.floor(fy);
        int ix1 = ix0 + 1;
        int iy1 = iy0 + 1;

        if (ix1 >= table.getNx()) ix1 = table.getNx() - 1;
        if (iy1 >= table.getNy()) iy1 = table.getNy() - 1;

        double q11 = table.sample(ix0, iy0);
        double q21 = table.sample(ix1, iy0);
        double q12 = table.sample(ix0, iy1);
        double q22 = table.sample(ix1, iy1);

        boolean has11 = !Double.isNaN(q11);
        boolean has21 = !Double.isNaN(q21);
        boolean has12 = !Double.isNaN(q12);
        boolean has22 = !Double.isNaN(q22);

        double x0 = minX + ix0 * spacing;
        double x1 = minX + ix1 * spacing;
        double y0 = minY + iy0 * spacing;
        double y1 = minY + iy1 * spacing;

        // ---- CASE 1: Full bilinear ----
        if (has11 && has21 && has12 && has22 && x1 != x0 && y1 != y0) {
            double denom = (x1 - x0) * (y1 - y0);
            double wx0 = (x1 - x);
            double wx1 = (x - x0);
            double wy0 = (y1 - y);
            double wy1 = (y - y0);

            return (q11 * wx0 * wy0 +
                    q21 * wx1 * wy0 +
                    q12 * wx0 * wy1 +
                    q22 * wx1 * wy1) / denom;
        }

        // ---- CASE 2: Linear horizontal ----
        if (has11 && has21 && x1 != x0)
            return q11 + (q21 - q11) * (x - x0) / (x1 - x0);

        if (has12 && has22 && x1 != x0)
            return q12 + (q22 - q12) * (x - x0) / (x1 - x0);

        // ---- CASE 3: Linear vertical ----
        if (has11 && has12 && y1 != y0)
            return q11 + (q12 - q11) * (y - y0) / (y1 - y0);

        if (has21 && has22 && y1 != y0)
            return q21 + (q22 - q21) * (y - y0) / (y1 - y0);

        // ---- CASE 4: One neighbor ----
        if (has11) return q11;
        if (has21) return q21;
        if (has12) return q12;
        if (has22) return q22;

        // ---- CASE 5: nearest neighbor fallback ----
        return nearest(x, y);
    }

    /**
     * Nearest neighbor search for missing regions
     */
    private double nearest(double x, double y) {
        double[][] grid = table.getGrid();
        double minX = table.getMinX();
        double minY = table.getMinY();
        double spacing = table.getSpacing();

        double bestDist = Double.POSITIVE_INFINITY;
        double bestAng = 0;
        boolean found = false;

        for (int ix = 0; ix < table.getNx(); ix++) {
            double gx = minX + ix * spacing;
            for (int iy = 0; iy < table.getNy(); iy++) {
                double ang = grid[ix][iy];
                if (Double.isNaN(ang)) continue;

                double gy = minY + iy * spacing;
                double dx = gx - x;
                double dy = gy - y;
                double dist = dx * dx + dy * dy;

                if (dist < bestDist) {
                    bestDist = dist;
                    bestAng = ang;
                    found = true;
                }
            }
        }

        return found ? bestAng : 0;
    }
}
