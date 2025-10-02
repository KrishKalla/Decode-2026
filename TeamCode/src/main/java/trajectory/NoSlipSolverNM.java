package trajectory;

import java.util.function.Function;

public class NoSlipSolverNM {
    private NoSlip template;

    public NoSlipSolverNM(NoSlip template) {
        this.template = template;
    }

    public SolverResult solve(double H_target, double D_target, double dt, double tMax) {

        // Cost function: sum of errors in max height and max distance
        Function<double[], Double> cost = params -> {
            double v0 = params[0];
            double d  = params[1];

            NoSlip sim = new NoSlip(
                    template.m, template.r, template.R,
                    d, v0,
                    template.rho, template.A, template.C_D,
                    template.g, template.C_rr
            );

            ExitState exit = sim.integrate(dt);

            double[] r0 = new double[]{0, 0, 0};
            double[] v0Vec = new double[]{
                    exit.vExit * Math.cos(exit.alphaExit),
                    exit.vExit * Math.sin(exit.alphaExit),
                    0
            };
            double[] omegaVec = new double[]{0, 0, 0};

            ExitTrajectory traj = new ExitTrajectory();
            ProjectileResult proj = traj.integrateProjectileRK4(
                    r0, v0Vec, omegaVec,
                    template.m, template.r, template.A, template.rho,
                    template.C_D, null,
                    template.g, 0, dt, tMax
            );

            return Math.abs(proj.H_max - H_target) + Math.abs(proj.D_max - D_target);
        };

        double[] x0 = new double[]{template.R * 5.0, template.R * 0.5}; // initial guess [v0, d]
        double[] opt = NelderMead.minimize(cost, x0, 1e-3, 200);

        double v0_opt = opt[0];
        double d_opt  = opt[1];

        // Final simulation
        NoSlip simFinal = new NoSlip(
                template.m, template.r, template.R,
                d_opt, v0_opt,
                template.rho, template.A, template.C_D,
                template.g, template.C_rr
        );
        ExitState exitFinal = simFinal.integrate(dt);

        double[] r0 = new double[]{0, 0, 0};
        double[] v0Vec = new double[]{
                exitFinal.vExit * Math.cos(exitFinal.alphaExit),
                exitFinal.vExit * Math.sin(exitFinal.alphaExit),
                0
        };

        ProjectileResult projFinal = new ExitTrajectory().integrateProjectileRK4(
                r0, v0Vec, new double[]{0, 0, 0},
                template.m, template.r, template.A, template.rho,
                template.C_D, null,
                template.g, 0, dt, tMax
        );

        SolverResult res = new SolverResult();
        res.v0 = v0_opt;
        res.omega0 = 0;
        res.d = d_opt;
        res.projectileResult = projFinal;
        return res;
    }
}
