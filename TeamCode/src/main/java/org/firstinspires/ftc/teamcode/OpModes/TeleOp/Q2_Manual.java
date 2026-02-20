//package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.experimental.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.intake;
//import org.firstinspires.ftc.teamcode.subsystems.shooter;
//import org.firstinspires.ftc.teamcode.util.LLHandler;
//import org.firstinspires.ftc.teamcode.util.constants;
//import org.firstinspires.ftc.teamcode.util.poseStorage;
//
//@Config
//@TeleOp(name = "Blue TeleOp Manuel", group = "1")
//public class Q2_Manual extends OpMode {
//    private Follower follower;
//    private intake intake;
//    private Turret turret;
//    private shooter shooter;
//    private LLHandler llhandler;
//
//    private ElapsedTime timer;
//    private int alliance = 1;
//
//    Thread thread;
//    Runnable r;
//
//
//    public static double MANUAL_HOOD;
//    public static boolean AUTO = true;
//
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.update();
//
//        intake = new intake();
//        turret = new Turret();
//        shooter = new shooter();
//        llhandler = new LLHandler(hardwareMap, alliance);
//
//        intake.init(hardwareMap);
//        turret.init(hardwareMap, llhandler, follower.poseTracker);
//        shooter.init(hardwareMap, llhandler);
//
//
//
//        timer = new ElapsedTime();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        r = new Runnable() {
//            @Override
//            public void run() {
//                while(true) {
//                    Pose goalPose = new Pose(poseStorage.BLUE_X, poseStorage.BLUE_Y);
//                    llhandler.poll();
//                    turret.update(goalPose);
//                    shooter.update();
//                }
//            }
//
//        };
//
//        //turret.zeroTurret();
//
//        thread = new Thread(r);
//
//    }
//
//    @Override
//    public void init_loop() {
//        updateTelemetry();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//        llhandler.start();
//        intake.setIntake(constants.INTAKE_PRESETS.OFF);
//
//        turret.zeroTurret();
//
//        shooter.flywheelPreset(constants.FLYWHEEL.ON);
//        shooter.hoodPreset(constants.HOOD.AUTO);
//        shooter.setStopper(true);
//
//        thread.start();
//
//        timer.reset();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true //robot centric
//        );
//
//        //Intake
//        if (gamepad1.right_trigger > 0.3) {
//            intake.setIntake(constants.INTAKE_PRESETS.ON);
//            shooter.setStopper(true);
//        } else if (gamepad1.left_trigger > 0.3) {
//            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
//        } else if (gamepad1.right_bumper) {
//            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
//        } else {
//            intake.setIntake(constants.INTAKE_PRESETS.OFF);
//        }
//
//        //Open Stopper
//        if (gamepad1.left_bumper) {
//            shooter.setStopper(false);
//        }
//
//        //GP2
//        if (gamepad2.left_trigger > 0.3) {
//            shooter.flywheelPreset(constants.FLYWHEEL.OFF);
//        }
//        if (gamepad2.right_trigger > 0.3) {
//            shooter.flywheelPreset(constants.FLYWHEEL.ON);
//        }
//
//        //Manual Controls for Hood
//        if (gamepad2.right_bumper) {
//            shooter.manual(1);
//        }
//        if (gamepad2.left_bumper) {
//            shooter.manual(-1);
//        }
//
//        //Close Zone Set points
//            //very close = square
//            if (gamepad2.square){
//                constants.shooter.TARGET_RPM=650;
//                constants.shooter.Hood_pos=0.24;
//
//            }
//            //Medium Range = triangle
//            if (gamepad2.triangle){
//                constants.shooter.TARGET_RPM=800;
//                constants.shooter.Hood_pos=0.78;
//            }
//            //Far Range= circle
//            if (gamepad2.circle){
//                constants.shooter.TARGET_RPM=800;
//                constants.shooter.Hood_pos=0.87;
//            }
//
//
//        //Far Zone Set points
//        if (gamepad2.cross){
//            constants.shooter.TARGET_RPM=900;
//            constants.shooter.Hood_pos=0.867;
//        }
//
//        //Fix Turret Pose Left
//        if (gamepad2.dpad_left){
//            turret.setTurretAngle(-90);
//        }
//
//        //Fix Turret Pose Right
//        if (gamepad2.dpad_right){
//            turret.setTurretAngle(90);
//        }
//
//        //Fix Turret Pose Middle
//        if (gamepad2.dpad_down){
//            turret.setTurretAngle(0);
//        }
//        updateTelemetry();
//    }
//
//    public void updateTelemetry() {
//        telemetry.addLine(follower.getPose().toString());
//        telemetry.addLine("≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡SHOOTER≡≡≡≡≡≡≡≡≡≡≡≡≡≡");
//        //ADD REGRESSION VALUE (IF WE HAVE ONE)
//        telemetry.addData("RPM", shooter.getRPM());
//        telemetry.addData("Hood Angle", shooter.getHoodAngle());
//        telemetry.update();
//    }
//}
