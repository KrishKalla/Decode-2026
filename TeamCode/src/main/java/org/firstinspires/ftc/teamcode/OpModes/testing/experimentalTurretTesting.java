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
import org.firstinspires.ftc.teamcode.subsystems.experimental.turret;
import org.firstinspires.ftc.teamcode.util.LLHandler;
import org.firstinspires.ftc.teamcode.util.SOTM;
import org.firstinspires.ftc.teamcode.util.storage;

@TeleOp(name = "Experimental Turret Testing", group = "testing")
@Config
public class experimentalTurretTesting extends OpMode {
    private turret turret;
    private LLHandler llhandler;
    private Follower follower;
    private int alliance = 1;
    private FtcDashboard dashboard;
    public static boolean manual = false;
    public static boolean RESET = false;
    public static double MANUAL = 0;

    Pose goalPose;

    public void init() {
        dashboard = FtcDashboard.getInstance();
        turret = new turret();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        llhandler = new LLHandler(hardwareMap, alliance);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();
        turret.init(hardwareMap, follower);
        turret.reset();
        goalPose = new Pose(storage.RED_X, storage.RED_Y);
        SOTM.setGoalPose(goalPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("encoder pos",turret.getDelta());
        telemetry.update();
    }

    public void start() {
        follower.startTeleopDrive();
        llhandler.start();
        turret.zeroTurret();
    }

    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,true);
        follower.update();
        if (manual) {
            turret.update(MANUAL);
        } else if (RESET) {
            turret.TEST_RESET_ONLY();
        }else {
//            llhandler.poll();
//            SOTM.calculate(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
//            turret.hardwareUpdate(turret.update(SOTM.getAdjustedGoal()));
        }
        turret.periodic();
//        telemetry.addData("pose", SOTM.getAdjustedGoal().toString());
//        telemetry.addData("vx", follower.getVelocity().getXComponent());
//        telemetry.addData("vy", follower.getVelocity().getYComponent());
        telemetry.addData("turret enc", turret.getDelta());
        telemetry.addData("turret target", turret.getTargetAngle());
        telemetry.addData("turret current", turret.getCurrentAngle());
        telemetry.addData("counter", storage.counter);
        telemetry.addData("zero", turret.ZERO);
        telemetry.addData("within boundary", boundaryDetection());
        telemetry.update();


        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay().setRotation(Math.toRadians(90)).setTranslation(72, -72);
        turret.drawTurret(canvas, packet);
        //Goal Tracking Pose
        canvas.setStroke("##00ff00");
        canvas.setFill("##00ff00");
        canvas.fillCircle(goalPose.getX(), goalPose.getY(), 2);

        canvas.setStroke("#FF0000");
        canvas.setFill("#FF0000");
        canvas.fillCircle(goalPose.getX(), goalPose.getY(), 2);


        packet.put("Target Angle", turret.getTargetAngle());
        packet.put("Current Angle", turret.getCurrentAngle());
        packet.put("Error", turret.getError());
        packet.put("Is Aimed", turret.isAimed());
        packet.put("Is Correcting", turret.isCorrecting());

        dashboard.sendTelemetryPacket(packet);
    }

    public boolean boundaryDetection() {
        double px = follower.getPose().getX();
        double py = follower.getPose().getY();

        double d1 = (px - 72) * (144-72) - (0 - 72) * (py - 72);
        double d2 = (px - 144) * (72 - 144) - (72 - 144) * (py - 144);
        double d3 = (px - 0) * (144 - 144) - (144 - 0) * (py - 144);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(hasNeg && hasPos);
    }
}