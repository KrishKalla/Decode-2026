package org.firstinspires.ftc.teamcode.OpModes.Auto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.CvType;

// Image processing methods
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ChooseSide extends OpenCvPipeline {
    // We use these Mats to avoid memory leaks by reusing them
    Mat hsvMat = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    public volatile double[] coordinate = {0.0, 0.0};
    List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to grayscale (Note: EOCV uses RGBA)
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGBA2GRAY);
        Scalar lowerGreen  = new Scalar(36, 50, 50);
        Scalar higherPurple = new Scalar(160, 255, 255);

        Core.inRange(hsvMat, lowerGreen, higherPurple, mask);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // 4. Optional: Draw contours back onto the live camera stream
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);

        double[] potentialCoordinate = findLargestContour(contours);

        if (potentialCoordinate != null) {
            coordinate = potentialCoordinate;
        }

        return input; // Return the Mat to be shown on the Robot Controller/Dashboard
    }

    private double[] findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largest = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largest = contour;
            }
        }

        Rect rect = Imgproc.boundingRect(largest);
        double x = rect.x + (rect.width / 2.0); // Center X
        double y = rect.y + (rect.height / 2.0); // Center Y

        double[] point = {x, y};

        return point;
    }
}