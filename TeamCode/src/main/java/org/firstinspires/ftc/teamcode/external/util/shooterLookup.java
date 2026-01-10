package org.firstinspires.ftc.teamcode.external.util;

import android.content.res.AssetManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

public class shooterLookup {
    /** Internal grid storing angles; NaN means missing */
    private final double[][] grid;

    /** Grid metadata */
    private final double minX;
    private final double minY;
    private final double spacing;
    private final int nx;
    private final int ny;

    /** Constructor — only called by loader */
    public shooterLookup(double[][] grid, double minX, double minY,
                      double spacing, int nx, int ny) {
        this.grid = grid;
        this.minX = minX;
        this.minY = minY;
        this.spacing = spacing;
        this.nx = nx;
        this.ny = ny;
    }

    /** Read a text file from assets */
    private static String readAsset(AssetManager assets, String file) throws Exception {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader br = new BufferedReader(
                new InputStreamReader(assets.open(file), StandardCharsets.UTF_8))) {
            char[] buf = new char[1024];
            int n;
            while ((n = br.read(buf)) != -1) sb.append(buf, 0, n);
        }
        return sb.toString();
    }

    /** Load JSON into a grid */
    public static shooterLookup loadFromAssets(HardwareMap hw, String file, double spacing) {
        try {
            String json = readAsset(hw.appContext.getAssets(), file);
            JSONArray arr = new JSONArray(json);

            double minX = Double.POSITIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY;
            double maxY = Double.NEGATIVE_INFINITY;

            // First pass: find bounds
            for (int i = 0; i < arr.length(); i++) {
                JSONObject o = arr.getJSONObject(i);
                double x = o.getDouble("x");
                double y = o.getDouble("y");
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }

            int nx = (int)Math.round((maxX - minX) / spacing) + 1;
            int ny = (int)Math.round((maxY - minY) / spacing) + 1;
            double[][] grid = new double[nx][ny];

            for (int ix=0; ix<nx; ix++)
                for (int iy=0; iy<ny; iy++)
                    grid[ix][iy] = Double.NaN;

            // Fill grid
            for (int i = 0; i < arr.length(); i++) {
                JSONObject o = arr.getJSONObject(i);
                double x = o.getDouble("x");
                double y = o.getDouble("y");
                double angle = o.getDouble("angle");

                int ix = (int) Math.round((x - minX) / spacing);
                int iy = (int) Math.round((y - minY) / spacing);

                if (ix >= 0 && iy >= 0 && ix < nx && iy < ny)
                    grid[ix][iy] = angle;
            }

            return new shooterLookup(grid, minX, minY, spacing, nx, ny);

        } catch (Exception e) {
            e.printStackTrace();
            // fallback empty table
            return new shooterLookup(new double[][]{{0}}, 0, 0, spacing, 1, 1);
        }
    }

    /** Sample: return NaN if outside or missing */
    public double sample(int ix, int iy) {
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny)
            return Double.NaN;
        return grid[ix][iy];
    }

    // Getters for interpolation class
    public double getMinX() { return minX; }
    public double getMinY() { return minY; }
    public double getSpacing() { return spacing; }
    public int getNx() { return nx; }
    public int getNy() { return ny; }
    public double[][] getGrid() { return grid; }
}