package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.experimental.Turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.constantsExperimental;

@Config
@TeleOp(name = "V1")
public class v1 extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private intake intake;
    private Turret turret;
    private shooter shooter;
    private LLHandler llhandler;
    private int alliance = 0;
    private ElapsedTime timer;
    Runnable r;
    private Thread thread;
    public static double MANUALHOOD = 0.2;
    private boolean shooting_state=false;

    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(72, 72, 0) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new intake();
        turret = new Turret();
        shooter = new shooter();
        llhandler = new LLHandler(hardwareMap, alliance);

        intake.init(hardwareMap);
        turret.init(hardwareMap, llhandler, follower.poseTracker);
        shooter.init(hardwareMap, llhandler);

        timer = new ElapsedTime();

        r = new Runnable() {
            @Override
            public void run() {
                while (true) {
                    Pose goalPose;
                    if (alliance == 1) {
                        goalPose = constantsExperimental.RED_GOAL;
                    } else {
                        goalPose = new Pose(constantsExperimental.BLUE_X, constantsExperimental.BLUE_Y);
                    }
                    turret.update(goalPose);
                    shooter.update();
                }
            }
        };

        thread = new Thread(r);
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
        thread.start();
        intake.setIntake(constants.INTAKE_PRESETS.OFF);
        turret.zeroTurret();
        shooter.hoodPreset(constants.HOOD.RESET);
        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        shooter.setStopper(true);
        //might need to change a bit here depending on how stuff works, not sure yet
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        shooter.update_constant();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x,
            true //robot centric
        );
        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
            shooting_state=false;
        }
        else if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
        }
        else{
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }

        if (gamepad1.right_bumper) {
            shooter.setStopper(true);
            shooting_state=false;
        }
        if (gamepad1.left_bumper) {
            shooter.setStopper(false);
            shooting_state=true;
        }

//        if (gamepad1.square) {
//            shooter.hoodPreset(constants.HOOD.AUTO);
//        }
//        if (gamepad1.circle) {
//            shooter.hoodPreset(constants.HOOD.RESET);
//        }
//        if (gamepad1.dpad_up) {
//            shooter.hoodPreset(constants.HOOD.MANUAL);
//            shooter.manual(1);
//        }
//        if (gamepad1.dpad_down) {
//            shooter.hoodPreset(constants.HOOD.MANUAL);
//            shooter.manual(-1);
//        }
        
//        if (gamepad1.dpad_right) {
//            turret.preset(constants.TURRET_PRESETS.MANUAL);
//            turret.manual(-1);
//        }
//        if (gamepad1.dpad_right) {
//            turret.preset(constants.TURRET_PRESETS.MANUAL);
//            turret.manual(1);
//        }



        addTelemetry("position", follower.getPose());
        addTelemetry("velocity", follower.getVelocity());

//        telemetry.addLine(intake.toString());
//        telemetry.addLine(turret.toString());
//        telemetry.addLine(shooter.toString());

        telemetryM.update();
        telemetry.update();
    }

    public void addTelemetry(String info, Object value) {
        telemetryM.debug(info, value);
        telemetry.addData(info, value);
    }

    @Override
    public void stop() {
        thread.interrupt();
    }
}
