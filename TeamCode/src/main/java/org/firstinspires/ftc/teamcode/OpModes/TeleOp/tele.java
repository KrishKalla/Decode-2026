package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.spindex;
import org.firstinspires.ftc.teamcode.utility.constants;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class tele extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private spindex spindex = new spindex();
    private intake intake = new intake();
    private shooter shooter = new shooter();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        spindex.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        if (gamepad1.squareWasPressed()) {
            spindex.preset(constants.SPINDEX.SPIN);
        }

        if (gamepad1.crossWasPressed()) {
            spindex.preset(constants.SPINDEX.PUSH);
        }

        if (gamepad1.right_trigger > 0.2) {
            intake.preset(constants.INTAKE.TAKEIN);
        }

        if (gamepad1.left_trigger > 0.2) {
            intake.preset(constants.INTAKE.REJECT);
        }

        if(gamepad1.dpad_down) {
            shooter.preset(constants.SHOOTER.SHOOTFAR);
        }

        else if (gamepad1.right_bumper) {
            shooter.preset(constants.SHOOTER.SHOOTSHORT);
        }

        else if (gamepad1.dpad_up) {
            shooter.preset(constants.SHOOTER.SHOOTSHORT);
        }

        else if(gamepad1.left_bumper){
            shooter.preset(constants.SHOOTER.OFF);
        }

        else if (gamepad1.dpad_left) {
            shooter.modulate(-1);
        }

        else if (gamepad1.dpad_right) {
            shooter.modulate(1);
        }

        else {
            intake.preset(constants.INTAKE.OFF);
        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("spindex Position", spindex.spindexServo.getPosition());
        telemetry.addData("spinindex", spindex.spinindex);
        telemetry.update();
    }
}