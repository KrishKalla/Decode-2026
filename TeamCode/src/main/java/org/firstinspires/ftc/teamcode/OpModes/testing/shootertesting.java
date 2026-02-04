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
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;
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
    private Follower follower;
    public static double MANUALHOOD = 0.875;
    public static double Delta_Hood = 0;

    private int alliance = 1;
    private LLHandler llhandler;

    public void init() {
        shooter = new shooter();
        intake = new intake();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        shooter.init(hardwareMap, llhandler);
        intake.init(hardwareMap);

        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();
    }

    @Override
    public void start() {
        ;
    }

    public void loop() {
        follower.update();
        shooter.update();
        shooter.update_constant();
        shooter.updateBatteryVoltage();

//        double shit = shooter.calculate();
//        shooter.motorLeft.setPower(shit);
//        shooter.motorRight.setPower(shit);


        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
        }
        if (gamepad1.left_bumper) {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
            shooter.setStopper(false);
        }
        if (gamepad1.right_bumper){
            shooter.setStopper(true);
        }
        telemetry.addLine(shooter.toString());
        telemetry.addData("RPM: ", shooter.getRPM());
        telemetry.addData("Target_RPM: ", constants.shooter.TARGET_RPM);
        telemetry.addData("Target_Hood: ", constants.shooter.Hood_pos);
        telemetry.addLine(intake.toString());
        telemetry.update();
        
    }
}
