package trajectory;

import java.util.Arrays;
import java.util.function.Function;

public class NelderMead {

    private static final double ALPHA = 1.0;
    private static final double GAMMA = 2.0;
    private static final double RHO   = 0.5;
    private static final double SIGMA = 0.5;

    /**
     * Minimize a function f: double[] -> double
     * x0 is initial guess, tol is stopping tolerance, maxIter is max iterations
     */
    public static double[] minimize(Function<double[], Double> f, double[] x0, double tol, int maxIter) {
        int n = x0.length;
        double[][] simplex = new double[n + 1][n];
        double[] fVals = new double[n + 1];

        // Initialize simplex
        simplex[0] = x0.clone();
        fVals[0] = f.apply(x0);
        for (int i = 1; i <= n; i++) {
            simplex[i] = x0.clone();
            simplex[i][i - 1] += 0.05 * (x0[i - 1] == 0 ? 1 : x0[i - 1]); // small offset
            fVals[i] = f.apply(simplex[i]);
        }

        for (int iter = 0; iter < maxIter; iter++) {
            // Sort simplex by fVals
            for (int i = 0; i <= n; i++) {
                for (int j = i + 1; j <= n; j++) {
                    if (fVals[j] < fVals[i]) {
                        double[] tempX = simplex[i]; simplex[i] = simplex[j]; simplex[j] = tempX;
                        double tempF = fVals[i]; fVals[i] = fVals[j]; fVals[j] = tempF;
                    }
                }
            }

            // Check convergence
            double fMean = 0;
            for (double val : fVals) fMean += val;
            fMean /= (n + 1);
            double fVar = 0;
            for (double val : fVals) fVar += Math.pow(val - fMean, 2);
            fVar /= (n + 1);
            if (Math.sqrt(fVar) < tol) break;

            // Centroid of all points except worst
            double[] xBar = new double[n];
            for (int i = 0; i < n; i++) xBar[i] = 0;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) xBar[j] += simplex[i][j];
            }
            for (int j = 0; j < n; j++) xBar[j] /= n;

            // Reflection
            double[] xr = new double[n];
            for (int j = 0; j < n; j++) xr[j] = xBar[j] + ALPHA * (xBar[j] - simplex[n][j]);
            double fr = f.apply(xr);

            if (fr < fVals[0]) {
                // Expansion
                double[] xe = new double[n];
                for (int j = 0; j < n; j++) xe[j] = xBar[j] + GAMMA * (xr[j] - xBar[j]);
                double fe = f.apply(xe);
                if (fe < fr) { simplex[n] = xe; fVals[n] = fe; }
                else { simplex[n] = xr; fVals[n] = fr; }
            } else if (fr < fVals[n - 1]) {
                simplex[n] = xr; fVals[n] = fr;
            } else {
                // Contraction
                double[] xc = new double[n];
                if (fr < fVals[n]) {
                    for (int j = 0; j < n; j++) xc[j] = xBar[j] + RHO * (xr[j] - xBar[j]);
                } else {
                    for (int j = 0; j < n; j++) xc[j] = xBar[j] + RHO * (simplex[n][j] - xBar[j]);
                }
                double fc = f.apply(xc);
                if (fc < fVals[n]) { simplex[n] = xc; fVals[n] = fc; }
                else {
                    // Shrink
                    for (int i = 1; i <= n; i++) {
                        for (int j = 0; j < n; j++) {
                            simplex[i][j] = simplex[0][j] + SIGMA * (simplex[i][j] - simplex[0][j]);
                        }
                        fVals[i] = f.apply(simplex[i]);
                    }
                }
            }
        }

        // Return best
        return simplex[0];
    }
}
