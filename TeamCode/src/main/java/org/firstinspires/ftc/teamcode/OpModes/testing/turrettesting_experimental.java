package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.experimental.Turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.constantsExperimental;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "turret testing")
@Config
public class turrettesting_experimental extends OpMode {
    private Turret turret;
    private LLHandler llhandler;
    private Follower follower;
    private Localizer localizer;
    private int alliance = 1;
    public void init() {
        turret = new Turret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();
        turret.init(hardwareMap, llhandler, follower.poseTracker);
        turret.zeroTurret();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Encoder Ticks", turret.encoder.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        llhandler.start();
        turret.zeroTurret();
    }

    public void loop() {
        Pose goalPose = constantsExperimental.BLUE_GOAL;
        follower.update();
        llhandler.poll();
        turret.update(goalPose);
        telemetry.addData("Current Pose", follower.getPose().toString());
        telemetry.addData("Encoder Ticks", turret.encoder.getCurrentPosition());
        telemetry.addData("TX", llhandler.getLatestResult()[3]);
        telemetry.addData("Current Turret Angle", "%.2f°", turret.getCurrentTurretAngle());
        telemetry.addData("Target Angle", "%.2f°", turret.getError());
        telemetry.addData("Is Aimed", turret.isAimedAtGoal(goalPose, 1.0));
        telemetry.addData("LLVALID", turret.getLLValid());
        telemetry.update();
    }
}
