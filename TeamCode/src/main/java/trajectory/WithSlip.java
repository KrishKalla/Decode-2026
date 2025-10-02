package trajectory;

public class WithSlip {
    double m, r, R, d, v0, omega0, rho, A, C_D, g, mu_s, mu_k, C_rr;
    public WithSlip(double m, double r, double R, double d, double v0, double omega0, double rho, double A, double C_D, double g, double mu_s, double mu_k, double C_rr) {
        this.m = m;
        this.r = r;
        this.R = R;
        this.d = d;
        this.v0 = v0;
        this.omega0 = omega0;
        this.rho = rho;
        this.A = A;
        this.C_D = C_D;
        this.g = g;
        this.mu_s = mu_s;
        this.mu_k = mu_k;
        this.C_rr = C_rr;
    }

    public ExitStateSlip integrateWithSlip(double dt) {
        double I = 2.0/3.0 * m * r * r;
        double phi_exit = d/r;

        double phi = 0.0;
        double v = v0;
        double omega = omega0;

        while(phi < phi_exit) {
            double[] k1 = derivativesWithSlip(phi, v, omega, I);
            double[] k2 = derivativesWithSlip(phi + 0.5 * dt * k1[0], v + 0.5 * dt * k1[1], omega + 0.5 * dt * k1[2], I);
            double[] k3 = derivativesWithSlip(phi + 0.5 * dt * k2[0], v + 0.5 * dt * k2[1], omega + 0.5 * dt * k2[2], I);
            double[] k4 = derivativesWithSlip(phi + dt * k3[0], v + dt * k3[1], omega + dt * k3[2], I);

            phi += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
            v += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
            omega += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
        }

        return new ExitStateSlip(phi, v, omega);
    }

    public double[] derivativesWithSlip(double phi ,double v, double omega, double I) {
        double N = this.m * this.g * Math.cos(phi) + this.m * v * v / this.R;
        double Fd = 0.5 * this.rho * this.C_D * this.A * v * v * Math.signum(v);
        double Frr = this.C_rr * N * Math.signum(v);
        double rel = v - this.r * omega;
        double eps = 1e-6;
        double Ff;
        if (Math.abs(rel) < eps) {
            double FfStatic = (I / this.r) * (0.0);
            if (Math.abs(FfStatic) <= this.mu_s * N) {
                Ff = FfStatic;
            } else {
                Ff = this.mu_k * N * Math.signum(rel);
            }
        } else {
            Ff = this.mu_k * N * Math.signum(rel);
        }

        double a = (-this.m * this.g * Math.sin(phi) - Fd - Frr - Ff) / this.m;
        double alpha = (this.r * Ff) / I;
        double dphi = v / this.R;
        return new double[]{dphi, a, alpha};
    }
}
