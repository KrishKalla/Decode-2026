package trajectory;

public class ExitStateSlip {
    public double v_exit;
    public double omega_exit;
    public double alpha_exit;

    public ExitStateSlip(double v_exit, double omega_exit, double alpha_exit) {
        this.v_exit = v_exit;
        this.omega_exit = omega_exit;
        this.alpha_exit = alpha_exit;
    }
}
