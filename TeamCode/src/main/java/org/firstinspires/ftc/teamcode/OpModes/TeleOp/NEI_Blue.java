package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

@Config
@TeleOp(name = "Blue Close TeleOp ", group = "1")
public class NEI_Blue extends OpMode {
    private Follower follower;
    private intake intake;
    private Turret turret;
    private shooter shooter;
    private LLHandler llhandler;

    private ElapsedTime timer;
    private int alliance = 1;
    private int loopCounter = 0;


    private volatile double servoUpdate;

    public static double MANUAL_TURRET = 0;
    public static boolean AUTO = true;
    public static boolean AUTO_AIM = true;

    private int Mode=0;//Short


    @Override
    public void init() {

        storage.BLUE_X = 6.7;

        follower = Constants.createFollower(hardwareMap);
        if (alliance == 1) {
            follower.setStartingPose(storage.lastBlueAutoPose);
        } else {
            follower.setStartingPose(storage.lastRedAutoPose);
        }

        follower.update();

        intake = new intake();
        turret = new Turret();
        shooter = new shooter();
        llhandler = new LLHandler(hardwareMap, alliance);
        llhandler.alliance(alliance);

        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);
        shooter.init(hardwareMap, llhandler);


        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        llhandler.start();
        intake.setIntake(constants.INTAKE_PRESETS.OFF);

        shooter.flywheelPreset(constants.FLYWHEEL.ON);
        shooter.hoodPreset(constants.HOOD.AUTO);
        AUTO = true;
        AUTO_AIM = true;
        shooter.setStopper(true);

        timer.reset();
    }

    @Override
    public void loop() {

        intake.update();

        timer.reset();

        // 1️⃣ Update drivetrain pose
        follower.update();

        // 3️⃣ Vision
        llhandler.poll();

        // 4️⃣ Shooter
        shooter.update();

        loopCounter++;

        if (AUTO && Mode == 0 && loopCounter % 2 == 0) {
            shooter.calculateParams();
        } else if (AUTO && Mode == 1) {
            shooter.far();
        }

        // 5️⃣ Turret targeting
        if (AUTO_AIM) {
            if (alliance == 1) {
                servoUpdate = turret.update(new Pose(storage.BLUE_X, storage.BLUE_Y));
            } else {
                servoUpdate = turret.update(new Pose(storage.RED_X, storage.RED_Y));
            }
        }

        // 6️⃣ Apply turret servo
        turret.hardwareUpdate(servoUpdate);

        // 7️⃣ Drive control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        // 8️⃣ Intake + driver controls
        handleDriverControls();

        // 9️⃣ Telemetry LAST
        updateTelemetry();
    }
    private void handleDriverControls() {
        //Intake
        if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
        } else if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
        } else if (gamepad1.right_bumper) {
            shooter.setStopper(false);
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
        }
        else if(gamepad1.right_stick_button){
            intake.setIntake(constants.INTAKE_PRESETS.GATE);
        }
        else {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }

        //Switch Modes
        if (gamepad2.right_trigger > 0.3) {
            Mode=0;//close
        }
        if (gamepad2.left_trigger> 0.3) {
            Mode=1;//far
        }

        //Far Mode Vs Close Mode
        if (gamepad2.right_bumper) {
            storage.BLUE_X+=2;
        }
        if (gamepad2.left_bumper) {
            storage.BLUE_X-=2;
        }

        //Close Zone Set points
        //very close = square
        if (gamepad2.square){
            AUTO = false;
            constants.shooter.TARGET_RPM=660;
            constants.shooter.Hood_pos=0.30;

        }
        //Medium Range = triangle
        if (gamepad2.triangle){
            AUTO = false;
            constants.shooter.TARGET_RPM=770;
            constants.shooter.Hood_pos=0.62;
        }
        //Far Range= circle
        if (gamepad2.circle){
            AUTO = false;
            constants.shooter.TARGET_RPM=850;
            constants.shooter.Hood_pos=0.74;
        }

        if (gamepad2.cross) {
            AUTO = true;
        }

        //Fix Turret Pose Left
        if (gamepad2.dpad_left){
            AUTO_AIM=false;
            turret.setManualAngle(135);
        }

        //Fix Turret Pose Right
        if (gamepad2.dpad_right){
            AUTO_AIM=false;
            turret.setManualAngle(-45);
        }

        //Fix Turret Pose Middle
        if (gamepad2.dpad_up){
            AUTO_AIM=false;
            turret.setManualAngle(0);
        }

        //TURN BACK INTO AUTO TURRET
        if (gamepad2.dpad_down){
            AUTO_AIM=true;
        }
    }



    public void updateTelemetry() {
        telemetry.addLine(follower.getPose().toString());
        telemetry.addLine("≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡SHOOTER≡≡≡≡≡≡≡≡≡≡≡≡≡≡");
        //ADD REGRESSION VALUE (IF WE HAVE ONE)
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Hood Angle", shooter.getHoodAngle());
        telemetry.addData("Auto_Aim",AUTO_AIM);
        telemetry.addData("Auto_Shooter", AUTO);
        telemetry.addData("Counter", storage.counter);
        telemetry.addData("Loop Timer", timer.milliseconds());
        telemetry.addData("New Servo Pos", servoUpdate);
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Goal Pose X",storage.BLUE_X);
        telemetry.addData("Transfer Powered",intake.getTransferCurrent());
        telemetry.update();
    }

}
