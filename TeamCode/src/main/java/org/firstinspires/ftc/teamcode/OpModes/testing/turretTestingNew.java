package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.poseStorage;

@TeleOp(name = "GOOD Turret Testing", group = "testing")
@Config
public class turretTestingNew extends OpMode {
    private Turret turret;
    private LLHandler llhandler;
    private Follower follower;
    private int alliance = 1;
    private FtcDashboard dashboard;
    public static boolean manual = false;
    public static double MANUAL = 90;

    public void init() {
        dashboard = FtcDashboard.getInstance();
        turret = new Turret();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();
        turret.init(hardwareMap, follower);
        turret.zeroTurret();
    }

    @Override
    public void init_loop() {
        telemetry.addData("encoder pos",turret.getEncoderPos());
        telemetry.update();
    }

    public void start() {
        llhandler.start();
        turret.zeroTurret();
    }

    public void loop() {
        Pose goalPose = new Pose(poseStorage.BLUE_X, poseStorage.BLUE_Y);
        follower.update();
        if (manual) {
            turret.update(MANUAL);
        } else {
            llhandler.poll();
            turret.update(goalPose);
        }


        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay().setRotation(Math.toRadians(90)).setTranslation(72, -72);
        turret.drawTurret(canvas, packet);
        //Goal Tracking Pose
        canvas.setStroke("#FF0000");
        canvas.setFill("#FF0000");
        canvas.fillCircle(goalPose.getX(), goalPose.getY(), 2);

        packet.put("Target Angle", turret.getTargetAngle());
        packet.put("Current Angle", turret.getCurrentAngle());
        packet.put("Error", turret.getError());
        packet.put("Is Aimed", turret.isAimed());

        dashboard.sendTelemetryPacket(packet);
    }
}