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
import org.firstinspires.ftc.teamcode.util.storage;

@TeleOp(name = "GOOD Turret Testing", group = "testing")
@Config
public class turretTestingNew extends OpMode {
    private Turret turret;
    private LLHandler llhandler;
    private Follower follower;
    private int alliance = 1;
    private FtcDashboard dashboard;
    public static boolean manual = false;
    public static boolean RESET = false;
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
        turret.reset();
    }

    @Override
    public void init_loop() {
        telemetry.addData("encoder pos",turret.getDelta());
        telemetry.update();
    }

    public void start() {
        llhandler.start();
        turret.zeroTurret();
    }

    public void loop() {
        Pose goalPose = new Pose(storage.BLUE_X, storage.BLUE_Y);
        follower.update();
        if (manual) {
            turret.hardwareUpdate(turret.update(MANUAL));
        } else if (RESET) {
            turret.TEST_RESET_ONLY();
         }else {
            llhandler.poll();
            turret.hardwareUpdate(turret.update(goalPose));
        }

        telemetry.addData("turret enc", turret.getDelta() );
        telemetry.addData("counter", storage.counter);
        telemetry.addData("zero", turret.ZERO);
        telemetry.update();


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