package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "shooter testing")
@Config
public class shootertesting extends OpMode {
    public static boolean ON = false;
    public static boolean STOPPER = false;
    private boolean automatedDrive;
    private shooter shooter;
    private intake intake;
    private Turret turret;
    private Follower follower;
    public static double MANUALHOOD = 0.875;
    public static double Delta_Hood = 0;

    private int alliance = 1;
    private LLHandler llhandler;

    public void init() {
        shooter = new shooter();
        intake = new intake();
        turret = new Turret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);

        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        turret.init(hardwareMap, follower);
    }

    @Override
    public void start() {
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        llhandler.alliance(1);
        llhandler.start();
        turret.setManualAngle(0);
    }

    public void loop() {
        follower.update();
        llhandler.poll();
        shooter.update();
        shooter.setHood(constants.shooter.Hood_pos);
//        shooter.update_constant();
        shooter.updateBatteryVoltage();

//        double shit = shooter.calculate();
//        shooter.motorLeft.setPower(shit);
//        shooter.motorRight.setPower(shit);


        if (gamepad1.right_trigger > 0.3||ON) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
        }
        if (gamepad1.left_bumper||!ON) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
        }
        if (gamepad1.right_bumper||STOPPER){
            shooter.setStopper(true);
        }
        else if (!STOPPER){
            shooter.setStopper(false);
        }
        telemetry.addLine(shooter.toString());
        telemetry.addData("RPM: ", shooter.getRPM());
        telemetry.addData("Target_RPM: ", constants.shooter.TARGET_RPM);
        telemetry.addData("Target_Hood: ", constants.shooter.Hood_pos);
        telemetry.addData("lldist", llhandler.getLatestResult()[2]);
        telemetry.addLine(intake.toString());
        telemetry.update();
        
    }
}
