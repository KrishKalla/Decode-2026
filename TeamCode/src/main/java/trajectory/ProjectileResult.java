package trajectory;
public class ProjectileResult {
    public double[] rFinal;    // final position {x, y, z}
    public double[] vFinal;    // final velocity {vx, vy, vz}
    public double tImpact;     // simulation time when projectile hits ground
    public double H_max;       // maximum vertical height reached
    public double D_max;       // horizontal distance at impact
}
