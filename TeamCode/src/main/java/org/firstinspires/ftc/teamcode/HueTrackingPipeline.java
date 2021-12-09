package org.firstinspires.ftc.teamcode;

import android.renderscript.Matrix3f;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    private double[] centerColorLab;
    //private double[] centerColorVanilla;

    /**
     * Lab values are
     * [L]ightness : range 0-100
     * [A] (green <-> red): range usually clamped to +- 128
     * [B] (blue <-> yellow): range usually clamped to +- 128
     */
    private double[] setpointLab = {204, 108, 178};

    /**
     * Difference is expressed as distance within a 3D colorspace
     * Horizontal axes: A/B
     * Vertical axis: L
     *
     * The practical bounds of this threshold are 0 to 375
     * 0 being royal blue and 375 being firetruck red, the two most opposite colors represented
     * by this colorspace
     */
    protected final float labDistanceThresholdSquared = 250;

    private double averageXPosition;
    private double averageYPosition;

    private boolean isPipelineReady = false;

    public HueTrackingPipeline() {
        this.setpointLab = new double[]{204, 108, 178};
    }

    public HueTrackingPipeline(double[] setpointLab) {
        this.setpointLab = setpointLab;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat originalImage = input.clone();
        int rows = input.rows();
        int cols = input.cols();

        /**
         *         WARNING: OpenCV is cancerous and converts 8-bit CIELAB values like this:
         *         L <- L*255/100, a <- a + 128, b <- b + 128
         *         This will make the OpenCV Limits of all 3 values 0 <-> 255
         */
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab); // Color Isolation
        centerColorLab = input.get((int)rows/2, (int)cols/2);
        
        input.convertTo(input, CvType.CV_32F);
        Core.subtract(input, new Scalar(setpointLab), input); // input - setpointLab

        Core.multiply(input, input, input); // input^2

        // Add all channels
        Core.reduce(input.reshape(1, cols * rows), input, 1, Core.REDUCE_SUM);
        input = input.reshape(1, rows);

        Core.inRange(input, new Scalar(0), new Scalar(labDistanceThresholdSquared), input);

        input.convertTo(input, CvType.CV_8U);
        Moments m = Imgproc.moments(input, true);
        double m10 = m.m10;
        double m01 = m.m01;
        double m00 = m.m00;

        double averageXPixelPosition = (m10 / m00);
        double averageYPixelPosition = (m01 / m00);
        averageXPosition = averageXPixelPosition /((double)(cols));
        averageYPosition = averageYPixelPosition /((double)(rows));

        if(debugMode) {
            Scalar color = new Scalar(127, 255, 127);
            Imgproc.line(originalImage, new Point(0, averageYPixelPosition), new Point(cols, averageYPixelPosition), color);
            Imgproc.line(originalImage, new Point(averageXPixelPosition, 0), new Point(averageXPixelPosition, rows), color);
        }

        isPipelineReady = true;

        input.release();
        //.convertTo(input, CvType.CV_8U);
        return originalImage;
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

    public double evaluateLabDistanceSquared(double[] lab) {
        // Convert dumbfuck values
        double[] labButNotStupid = {100*lab[0]/255, lab[1] - 128, lab[2] - 128};

        return Math.pow(labButNotStupid[1] - setpointLab[1], 2) + Math.pow(labButNotStupid[2] - setpointLab[2], 2) + Math.pow(labButNotStupid[0] - setpointLab[0], 2);
    }

    public void setSetpointLab(double[] setpointLab) {
        this.setpointLab = setpointLab;
    }

    public double[] getSetpointLab() {
        return this.setpointLab;
    }
}
