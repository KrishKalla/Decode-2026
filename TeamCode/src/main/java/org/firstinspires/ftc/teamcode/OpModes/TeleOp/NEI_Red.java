package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constants;
import org.firstinspires.ftc.teamcode.util.storage;

@Config
@TeleOp(name = "Red Close TeleOp ", group = "1")
public class NEI_Red extends OpMode {
    private Follower follower;
    private intake intake;
    private turret turret;
    private shooter shooter;
    private LLHandler llhandler;

    private ElapsedTime timer;
    private ElapsedTime threadTimer = new ElapsedTime();
    private int alliance = 0;

    Thread thread;
    Runnable r;

    private volatile double servoUpdate;

    public static double MANUAL_TURRET = 0;
    public static boolean AUTO = true;
    public static boolean AUTO_AIM = true;
    private final Pose goalpose = new Pose(storage.RED_X, storage.RED_Y);

    private int Mode=0;//Short

    //Heading Lock
    double targetHeading = Math.toRadians(21); // Radians
    public static double heading_P=0.7;
    public static double heading_D=0.0000001;
    public static double heading_F=0.001;
    public static PIDFController controller = new PIDFController(new PIDFCoefficients(heading_P, 0, heading_D, heading_F));
    public static boolean headingLock = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (alliance == 1) {
            follower.setStartingPose(storage.lastBlueAutoPose);
        } else {
            follower.setStartingPose(storage.lastRedAutoPose);
        }

        follower.update();

        intake = new intake();
        turret = new turret();
        shooter = new shooter();
        llhandler = new LLHandler(hardwareMap, alliance);
        llhandler.alliance(alliance);

        intake.init(hardwareMap);
        turret.init(hardwareMap, follower);
        shooter.init(hardwareMap, llhandler);



        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        r = new Runnable() {
            @Override
            public void run() {
                while(true) {
                    threadTimer.reset();
                    llhandler.poll();
                    shooter.update();
                    shooter.updateBatteryVoltage();
                    if (AUTO && Mode==0) {
                        shooter.calculateParams();
                    }
                }
            }
        };

        thread = new Thread(r);
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

        thread.start();

        timer.reset();
    }

    @Override
    public void loop() {

        controller = new PIDFController(new PIDFCoefficients(heading_P, 0, heading_D, heading_F));
        controller.updateError(getHeadingError());

        if (headingLock)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(),true);
        else
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.9,true);


        intake.update();
        timer.reset();
        follower.update();
        if(AUTO_AIM) {
            turret.update(goalpose);
            turret.periodic();
        }

        //Intake

        if(gamepad1.right_stick_button){
            intake.setIntake(constants.INTAKE_PRESETS.GATE);
        } else if (gamepad1.right_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.ON);
            shooter.setStopper(true);
        } else if (gamepad1.left_trigger > 0.3) {
            intake.setIntake(constants.INTAKE_PRESETS.REJECT);
            intake.blocked=false;
        } else if (gamepad1.right_bumper) {
            shooter.setStopper(false);
            intake.setIntake(constants.INTAKE_PRESETS.TRANSFERING);
        } else {
            intake.setIntake(constants.INTAKE_PRESETS.OFF);
        }



        //Open Stopper
        if (gamepad1.left_bumper || ((follower.getPose().getY()>=80)&&(follower.getPose().getY()>=follower.getPose().getX()))){
            shooter.setStopper(false);
        }


        //Switch Modes
        if (gamepad1.right_stick_button) {
            targetHeading=Math.toRadians(25);
            headingLock=true;
            shooter.setStopper(true);
        }
        else if (gamepad2.right_trigger> 0.3) {
            targetHeading=Math.toRadians(0);
            headingLock=true;
        }
        else{
            headingLock=false;
        }



        //Human Player Zone Reloc
        if(gamepad2.right_stick_button){
            follower.setPose(new Pose(6.55,8,Math.toRadians(-90)));
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
            turret.setManualAngle(-135);
        }

        //Fix Turret Pose Right
        if (gamepad2.dpad_right){
            AUTO_AIM=false;
            turret.setManualAngle(45);
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
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addLine(follower.getPose().toString());

        telemetry.addLine("≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡INTAKE≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡");
        telemetry.addLine(intake.toString());
        telemetry.addLine("≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡SHOOTER≡≡≡≡≡≡≡≡≡≡≡≡≡≡");
        telemetry.addData("AUTO AIM ACTIVE",AUTO_AIM);
        telemetry.addData("REGRESSION ACTIVE", AUTO);
        telemetry.addData("EMA", shooter.ema);
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("Hood Angle", shooter.getHoodAngle());
        telemetry.addData("Counter", storage.counter);
        telemetry.addData("Turret Error", turret.getError());
        telemetry.addData("Transfer Powered",intake.getTransferCurrent());

        telemetry.addData("Heading Error", getHeadingError());
        telemetry.addData("Heading Output", controller.run());
        telemetry.addData("Current Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
        telemetry.update();
    }

    public double getHeadingError() {
        return MathFunctions.getTurnDirection(follower.getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(),targetHeading);
    }

    @Override
    public void stop(){
        thread.interrupt();
    }
}
