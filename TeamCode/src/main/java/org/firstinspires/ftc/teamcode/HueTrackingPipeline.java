package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * The CIELAB (L*a*b*) colorspace is being used because
 * the nonlinear relations for L*, a*, and b* are intended to mimic the nonlinear
 * response of the eye. Furthermore, uniform changes of components in the L*a*b* color space
 * aim to correspond to uniform changes in perceived color, so the relative perceptual differences
 * between any two colors in L*a*b* can be approximated by treating each color as a point in a
 * three-dimensional space (with three components: L*, a*, b*) and taking the Euclidean distance between them.
 */

public class HueTrackingPipeline extends OpenCvPipeline {
    final boolean debugMode = false;

    List<Mat> channels = new ArrayList<>();
    Mat originalImage = new Mat();

    double m10;
    double m01;
    double m00;
    double cols;

    double[] centerColorLab;
    double[] centerColorVanilla;

    /**
     * Lab values are
     * [L]ightness : range 0-100
     * [A] (green <-> red): range usually clamped to +- 128
     * [B] (blue <-> yellow): range usually clamped to +- 128
     */
    double[] setpointLab = {80, -20, 50};

    /**
     * Difference is expressed as distance within a 3D colorspace
     * Horizontal axes: A/B
     * Vertical axis: L
     *
     * The practical bounds of this threshold are 0 to 375
     * 0 being royal blue and 375 being firetruck red, the two most opposite colors represented
     * by this colorspace
     */
    float labDistanceThreshold = 20;

    double averageXPosition;
    double averageXPixelPosition;
    double averageYPosition;
    double averageYPixelPosition;

    boolean isPipelineReady = false;

    @Override
    public Mat processFrame(Mat input) {
        // TODO: Someone on stackoverflow said that this ^ "Mat input" value is BGR not RGB so now im paranoid and I need to double check this before I do hundreds of hours of work just to find out that its BGR not RGB
        originalImage = input.clone();

        centerColorVanilla = input.get((int)input.rows()/2, (int)input.cols()/2);

        /**
         *         WARNING: OpenCV is cancerous and converts 8-bit CIELAB values like this:
         *         L <- L*255/100, a <- a + 128, b <- b + 128
         *         This will make the OpenCV Limits of all 3 values 0 <-> 255
         */
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab); // Color Isolation
        centerColorLab = input.get((int)input.rows()/2, (int)input.cols()/2);

        // Loop through pixels and evaluate saturation and value
        for(int x = 0; x < input.cols(); x++) {
            for(int y = 0; y < input.rows(); y++) {
                if(evaluateLabDistance(input.get(y, x)) < labDistanceThreshold) {
                    input.put(y, x, 255d,255d,255d);
                } else {
                    input.put(y, x, 0d,0d,0d);
                }
            }
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        Core.multiply(input, new Scalar(255), input);

        Moments m = Imgproc.moments(input);
        m10 = m.m10;
        m01 = m.m01;
        m00 = m.m00;
        cols = input.cols();

        averageXPixelPosition = (m10/m00);
        averageYPixelPosition = (m01/m00);
        averageXPosition = averageXPixelPosition/((double)(input.cols()));
        averageYPosition = averageYPixelPosition/((double)(input.rows()));

        if(debugMode) {
            double[] color = {0, 255, 0, 0};
            //double color = 255;
            for (int y = 0; y < input.rows(); y++) {
                originalImage.put(y, (int) averageXPixelPosition, color);
            }

            for (int x = 0; x < input.cols(); x++) {
                originalImage.put((int) averageYPixelPosition, x, color);
            }
        }

        isPipelineReady = true;

        return input;
    }

    public Double getAverageXPosition() {
        return averageXPosition;
    }

    public Double getAverageYPosition() {
        return averageYPosition;
    }

    public double[] getCenterColorLab() {
        return centerColorLab;
    }

    public double[] getCenterColorVanilla() {
        return centerColorVanilla;
    }

    public boolean isPipelineReady() {
        return isPipelineReady;
    }

    // Evaluate saturation and value to be over a certain slope
    @Deprecated
    public boolean evaluateSatVal(double[] hsv) {
        final double satValSlope = 30/65; // Slope
        final double satPoint = 35;
        final double valPoint = 100;
        double lowestPossibleValue = (satValSlope * (hsv[1] - ((satPoint/100)*255))) + ((valPoint/100)*255); // Point slope form

        if(lowestPossibleValue > hsv[2]) {
            return false;
        }
        return true;
    }

    public double evaluateLabDistance(double[] lab) {
        // Convert dumbfuck values
        double[] labButNotStupid = {100*lab[0]/255, lab[1] - 128, lab[2] - 128};


        return Math.sqrt( Math.pow(labButNotStupid[1] - setpointLab[1], 2) + Math.pow(labButNotStupid[2] - setpointLab[2], 2) + Math.pow(labButNotStupid[0] - setpointLab[0], 2));
    }
}
