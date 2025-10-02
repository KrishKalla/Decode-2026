package trajectory;

public class NoSlip {
    double m, r, R, d, v0, rho, A, C_D, g,C_rr;
    public NoSlip(double m, double r, double R, double d, double v0, double rho, double A, double C_D, double g, double C_rr) {
        this.m = m;
        this.r = r;
        this.R = R;
        this.d = d;
        this.v0 = v0;
        this.rho = rho;
        this.A = A;
        this.C_D = C_D;
        this.g = g;
        this.C_rr = C_rr;
    }

    public ExitState integrate(double dt) {
        double I = 2.0/3.0 * this.m * this.r * this.r;
        double m_eff = this.m + I/(this.r*this.r);

        double phi_exit = this.d/this.r;
        double phi = 0.0;
        double omega = this.v0/this.R;

        while(phi < phi_exit) {
            double[] k1 = derivativesNoSlip(phi, omega, m_eff);
            double[] k2 = derivativesNoSlip(phi + 0.5 * dt * k1[0], omega + 0.5 * dt * k1[1], m_eff);
            double[] k3 = derivativesNoSlip(phi + 0.5 * dt * k2[0], omega + 0.5 * dt * k2[1],m_eff);
            double[] k4 = derivativesNoSlip(phi + dt * k3[0], omega + dt * k3[1], m_eff);
            phi += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
            omega += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
        }

        double v_exit = this.R * omega;
        double alpha_exit = Math.PI/2.0 - phi;
        return new ExitState(v_exit, alpha_exit);
    }

    public double[] derivativesNoSlip(double phi, double omega, double m_eff) {
        double v = this.R * omega;
        double Fd = 0.5 * this.rho * this.C_D * this.A * v * v * Math.signum(v);
        double N = m * g * Math.cos(phi) + m * v * v / R;
        double Frr = C_rr * N * Math.signum(v);
        double alpha = (-m * g * Math.sin(phi) - Fd - Frr) / (m_eff * R);

        return new double[]{omega, alpha};
    }
}
