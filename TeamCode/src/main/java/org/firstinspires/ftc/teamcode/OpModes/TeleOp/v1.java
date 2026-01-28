package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;

@TeleOp(name = "V1")
public class v1 extends OpMode {
    private Follower follower;
    public static Pose startingPose;

    private intake intake;
    private turret turret;
    private shooter shooter;
    private LLHandler llhandler;
    private int alliance = 0;
    private ElapsedTime timer;
    Runnable r;

    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new intake();
        turret = new turret();
        shooter = new shooter();
        llhandler = new LLHandler(hardwareMap, alliance);

        intake.init(hardwareMap);
        turret.init(hardwareMap, llhandler);
        shooter.init(hardwareMap, llhandler);

        timer = new ElapsedTime();

        r = new Runnable() {
            @Override
            public void run() {
                if (turret.state.equals("AUTO")) {
                    turret.update();
                }
                if (shooter.hoodState.equals("AUTO")) {
                    shooter.update();
                }
            }
        };
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) {
            alliance = 1;
        }
        if (gamepad1.left_bumper) {
            alliance = 0;
        }
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        timer.reset();
        llhandler.alliance(alliance);
        llhandler.start();
        r.run();
        intake.setIntake(constants.INTAKE_PRESETS.OFF);
        turret.preset(constants.TURRET_PRESETS.RESET);
        shooter.hoodPreset(constants.HOOD.RESET);
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        shooter.setStopper(true);
        //might need to change a bit here depending on how stuff works, not sure yet
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x,
            false //field centric
        );

        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        else if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
        }
        else{
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }

        if (gamepad1.right_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.ON);
        }
        if (gamepad1.left_bumper) {
            shooter.flywheelPreset(constants.FLYWHEEL.OFF);
        }

        if (gamepad1.square) {
            shooter.hoodPreset(constants.HOOD.AUTO);
        }
        if (gamepad1.circle) {
            shooter.hoodPreset(constants.HOOD.RESET);
        }
        if (gamepad1.dpad_up) {
            shooter.hoodPreset(constants.HOOD.MANUAL);
            shooter.manual(1);
        }
        if (gamepad1.dpad_down) {
            shooter.hoodPreset(constants.HOOD.MANUAL);
            shooter.manual(-1);
        }

        if (gamepad1.cross) {
            turret.preset(constants.TURRET_PRESETS.AUTO);
        }
        if (gamepad1.triangle) {
            turret.preset(constants.TURRET_PRESETS.RESET);
        }
//        if (gamepad1.dpad_right) {
//            turret.preset(constants.TURRET_PRESETS.MANUAL);
//            turret.manual(-1);
//        }
//        if (gamepad1.dpad_left) {
//            turret.preset(constants.TURRET_PRESETS.MANUAL);
//            turret.manual(1);
//        }

        if (gamepad1.dpad_left) {
            shooter.setStopper(false);
        }
        if (gamepad1.dpad_right) {
            shooter.setStopper(true);
        }



        addTelemetry("position", follower.getPose());
        addTelemetry("velocity", follower.getVelocity());

        telemetry.addLine(intake.toString());
        telemetry.addLine(turret.toString());
        telemetry.addLine(shooter.toString());

        telemetry.update();
    }

    public void addTelemetry(String info, Object value) {
        telemetry.addData(info, value);
    }
}
