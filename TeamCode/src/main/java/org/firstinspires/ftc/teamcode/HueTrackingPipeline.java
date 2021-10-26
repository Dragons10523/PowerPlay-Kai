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

    List<Mat> channels = new ArrayList<>();

    double m10;
    double m00;
    double cols;
    boolean quality = false;

    double[] centerColor;

    /**
     * Lab values are
     * [L]ightness : range 0-100
     * [A] (green <-> red): range usually clamped to +- 128
     * [B] (blue <-> yellow): range usually clamped to +- 128
     */
    double[] setpointLab = {82, -21, 128};

    /**
     * Difference is expressed as distance within a 3D colorspace
     * Horizontal axes: A/B
     * Vertical axis: L
     *
     * The practical bounds of this threshold are 0 to 375
     * 0 being royal blue and 375 being firetruck red, the two most opposite colors represented
     * by this colorspace
     */
    float labDistanceThreshold = 50;

    double averageXPosition;
    double averageXPixelPosition;

    @Override
    public Mat processFrame(Mat input) {

        // TODO: Verify that OpenCV L*A*B* values are clamped to the standard values and not expressed as percentages
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab); // Color Isolation
        centerColor = input.get((int)input.rows()/2, (int)input.cols()/2);

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
        m00 = m.m00;
        cols = input.cols();

        quality = m00 > 4000;

        averageXPixelPosition = (m.m10/m.m00);
        averageXPosition = averageXPixelPosition/((double)(input.cols()));

        for(int y = 0; y < input.rows(); y++) {
            double[] color = {0, 255, 0};
            input.put(y, (int)averageXPixelPosition, color);
        }

        return input;
    }

    public Double getAverageXPosition() {
        return averageXPosition;
    }

    public double[] getCenterColor() {
        return centerColor;
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
        return Math.sqrt( Math.pow(lab[1] - setpointLab[1], 2) + Math.pow(lab[2] - setpointLab[2], 2) + Math.pow(lab[0] - setpointLab[0], 2));
    }
}
