package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RingPipeline extends OpenCvPipeline {
    public static double H_MIN = 100;
    public static double H_MAX = 113;
    public static double S_MIN = 40;
    public static double V_MIN = 0.0;

    public static int LEFT_BOUND = 120;
    public static int UPPER_BOUND = 38;
    public static int WIDTH = 55;
    public static int HEIGHT = 45;

    public static double FOUR_THRESHOLD = 0.8;
    public static double ONE_THRESHOLD = 0.4;

    private double[] currentHSV = { 0.0, 0.0, 0.0 };

    public enum RingConfiguration {
        NULL,
        NONE,
        ONE,
        FOUR
    }

    private double percentage = 0.0;
    private RingConfiguration ringConfiguration = RingConfiguration.NULL;

    private Mat processingMat = new Mat();
    private Mat hsvMat = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        if (input != null) {
            Imgproc.blur(input, processingMat, new Size(20, 20));
            Imgproc.cvtColor(processingMat, hsvMat, Imgproc.COLOR_BGR2HSV);
            currentHSV = hsvMat.get(hsvMat.rows() / 2, hsvMat.cols() / 2);

            Core.inRange(hsvMat, new Scalar(H_MIN, S_MIN, V_MIN), new Scalar(H_MAX, 255, 255), hsvMat);
            Rect rect = new Rect(input.cols() / 2, input.rows() / 2, 20, 20);
            Imgproc.rectangle(hsvMat, rect, new Scalar(0.0, 255.0, 0.0));

            Rect ringRect = new Rect(LEFT_BOUND, UPPER_BOUND, WIDTH, HEIGHT);
            Imgproc.rectangle(input, ringRect, new Scalar(255.0, 0.0, 0.0));

            Mat subMat = hsvMat.submat(ringRect);
            int white = 0;
            int black = 0;
            for (int i = 0; i < subMat.rows(); i++) {
                for (int j = 0; j < subMat.cols(); j++) {
                    if (subMat.get(i, j)[0] > 0.0) {
                        white++;
                    } else {
                        black++;
                    }
                }
            }

            percentage = (double)white / ((double)white + (double)black);
            if (percentage >= FOUR_THRESHOLD) {
                ringConfiguration = RingConfiguration.FOUR;
            } else if (percentage >= ONE_THRESHOLD) {
                ringConfiguration = RingConfiguration.ONE;
            } else {
                ringConfiguration = RingConfiguration.NONE;
            }
        }
        return input;
    }

    public double[] getCurrentHSV() {
        return currentHSV;
    }

    public RingConfiguration getRingConfiguration() {
        return ringConfiguration;
    }

    public double getPercentage() {
        return percentage;
    }
}
