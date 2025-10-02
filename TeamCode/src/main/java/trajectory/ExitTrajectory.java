package trajectory;

import java.util.function.Function;

public class ExitTrajectory {

    public ProjectileResult integrateProjectileRK4(
            double[] r0,
            double[] v0,
            double[] omegaVec,
            double m, double radius, double A,
            double rho, double C_D,
            Function<Double, Double> C_L_of_S,
            double g, double yGround,
            double dt, double tMax) {

        double[] r = r0.clone();
        double[] v = v0.clone();
        ProjectileResult result = new ProjectileResult();

        double H_max = r[1]; // track maximum height
        double D_max = r[0]; // track maximum horizontal distance

        for (double t = 0; t < tMax; t += dt) {
            // RK4 integration
            double[] k1r = v.clone();
            double[] k1v = accel(v, omegaVec, m, radius, A, rho, C_D, C_L_of_S, g);

            double[] vTemp = vecAdd(v, vecScale(k1v, dt / 2));
            double[] k2r = vecAdd(v, vecScale(k1v, dt / 2));
            double[] k2v = accel(vTemp, omegaVec, m, radius, A, rho, C_D, C_L_of_S, g);

            vTemp = vecAdd(v, vecScale(k2v, dt / 2));
            double[] k3r = vecAdd(v, vecScale(k2v, dt / 2));
            double[] k3v = accel(vTemp, omegaVec, m, radius, A, rho, C_D, C_L_of_S, g);

            vTemp = vecAdd(v, vecScale(k3v, dt));
            double[] k4r = vecAdd(v, vecScale(k3v, dt));
            double[] k4v = accel(vTemp, omegaVec, m, radius, A, rho, C_D, C_L_of_S, g);

            r = vecAdd(r, vecScale(vecAdd4(k1r, k2r, k3r, k4r), dt / 6.0));
            v = vecAdd(v, vecScale(vecAdd4(k1v, k2v, k3v, k4v), dt / 6.0));

            // update maxima
            if (r[1] > H_max) H_max = r[1];
            if (r[0] > D_max) D_max = r[0];

            // check ground impact
            if (r[1] <= yGround) {
                result.rFinal = r.clone();
                result.vFinal = v.clone();
                result.tImpact = t;
                result.H_max = H_max;
                result.D_max = D_max;
                return result;
            }
        }

        // if simulation ends without hitting ground
        result.rFinal = r.clone();
        result.vFinal = v.clone();
        result.tImpact = tMax;
        result.H_max = H_max;
        result.D_max = D_max;
        return result;
    }

    private double[] accel(double[] v, double[] omega, double m, double r,
                           double A, double rho, double C_D,
                           Function<Double, Double> C_L_of_S, double g) {

        double vMag = norm(v);
        if (vMag < 1e-9) return new double[]{0, -g, 0};

        // Drag
        double FdMag = 0.5 * rho * C_D * A * vMag * vMag;
        double[] Fd = vecScale(v, -FdMag / vMag);

        // Magnus lift
        double S = omega == null ? 0 : (r * norm(omega) / vMag);
        double C_L = (C_L_of_S != null) ? C_L_of_S.apply(S) : 0.2 * S;
        double[] Fm = cross(omega, v);
        double FmMag = 0.5 * rho * A * C_L * vMag * vMag;
        Fm = norm(Fm) > 1e-9 ? vecScale(Fm, FmMag / norm(Fm)) : new double[]{0, 0, 0};

        // Gravity
        double[] Fg = new double[]{0, -m * g, 0};

        // Total acceleration
        double[] F = vecAdd(Fg, vecAdd(Fd, Fm));
        return vecScale(F, 1.0 / m);
    }

    // --- vector helpers ---
    double[] vecAdd(double[] a, double[] b) {
        return new double[]{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    double[] vecScale(double[] a, double s) {
        return new double[]{a[0] * s, a[1] * s, a[2] * s};
    }

    double[] vecAdd4(double[] a, double[] b, double[] c, double[] d) {
        return new double[]{
                a[0] + 2 * b[0] + 2 * c[0] + d[0],
                a[1] + 2 * b[1] + 2 * c[1] + d[1],
                a[2] + 2 * b[2] + 2 * c[2] + d[2]
        };
    }

    double norm(double[] a) {
        return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    }

    double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }
}
