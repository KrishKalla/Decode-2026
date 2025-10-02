package trajectory;

import java.util.function.Function;

public class WithSlipSolverNM {
    private WithSlip template;

    public WithSlipSolverNM(WithSlip template) {
        this.template = template;
    }

    public SolverResult solve(double H_target, double D_target, double dt, double tMax) {

        // Cost function over [v0, omega0, d]
        Function<double[], Double> cost = params -> {
            double v0     = params[0];
            double omega0 = params[1];
            double d      = params[2];

            WithSlip sim = new WithSlip(
                    template.m, template.r, template.R,
                    d, v0, omega0,
                    template.rho, template.A, template.C_D,
                    template.g, template.mu_s, template.mu_k, template.C_rr
            );

            ExitStateSlip exit = sim.integrateWithSlip(dt);

            double[] r0 = new double[]{0, 0, 0};
            double[] v0Vec = new double[]{
                    exit.v_exit * Math.cos(exit.alpha_exit),
                    exit.v_exit * Math.sin(exit.alpha_exit),
                    0
            };
            double[] omegaVec = new double[]{0, 0, exit.omega_exit};

            ExitTrajectory traj = new ExitTrajectory();
            ProjectileResult proj = traj.integrateProjectileRK4(
                    r0, v0Vec, omegaVec,
                    template.m, template.r, template.A, template.rho,
                    template.C_D, null,
                    template.g, 0, dt, tMax
            );

            return Math.abs(proj.H_max - H_target) + Math.abs(proj.D_max - D_target);
        };

        double[] x0 = new double[]{template.R * 5.0, 10.0, template.R * 0.5}; // initial guess [v0, omega0, d]
        double[] opt = NelderMead.minimize(cost, x0, 1e-3, 300);

        double v0_opt     = opt[0];
        double omega0_opt = opt[1];
        double d_opt      = opt[2];

        WithSlip simFinal = new WithSlip(
                template.m, template.r, template.R,
                d_opt, v0_opt, omega0_opt,
                template.rho, template.A, template.C_D,
                template.g, template.mu_s, template.mu_k, template.C_rr
        );

        ExitStateSlip exitFinal = simFinal.integrateWithSlip(dt);

        double[] r0 = new double[]{0, 0, 0};
        double[] v0Vec = new double[]{
                exitFinal.v_exit * Math.cos(exitFinal.alpha_exit),
                exitFinal.v_exit * Math.sin(exitFinal.alpha_exit),
                0
        };
        double[] omegaVec = new double[]{0, 0, exitFinal.omega_exit};

        ProjectileResult projFinal = new ExitTrajectory().integrateProjectileRK4(
                r0, v0Vec, omegaVec,
                template.m, template.r, template.A, template.rho,
                template.C_D, null,
                template.g, 0, dt, tMax
        );

        SolverResult res = new SolverResult();
        res.v0 = v0_opt;
        res.omega0 = omega0_opt;
        res.d = d_opt;
        res.projectileResult = projFinal;
        return res;
    }
}
