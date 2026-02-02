package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LLHandler {
    private Limelight3A limelight;
    private String handler;
    private LLResult result;

    private double[] latestResult;

    //BLUE = 0, RED = 1
    public LLHandler(HardwareMap map, int alliance) {
        limelight = map.get(Limelight3A.class, "ll");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(alliance);
        handler = "INIT";
    }

    public void start() {
        limelight.start();
        poll();
        handler = "ON";
    }
    public double[] poll() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> tags = result.getFiducialResults();
            FiducialResult tag = tags.get(0);

            Pose3D goal = tag.getTargetPoseCameraSpace();

            double x = goal.getPosition().x;
            double y = -goal.getPosition().y;
            double z = goal.getPosition().z;
            double tx = -result.getTx();

            latestResult = new double[] {x, y, z, tx};
            return new double[] {x, y, z, tx};
        }
        else {
            latestResult = new double[] {-1001, -1001, -1001, -1001};
            return new double[] {-1001, -1001, -1001, -1001};
        }
    }

    public void alliance(int a) {
        limelight.stop();
        limelight.pipelineSwitch(a);
    }

    public void stop() {
        limelight.stop();
        handler = "OFF";
    }

    public double[] getLatestResult() {
        return latestResult;
    }

    @NonNull
    public String toString() {
        return handler;
    }

    public LLResult getResult() {
        return result;
    }
}
