package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
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
    private boolean renderLines = true;

    private double[] centerColorLab;

    /**
     * Lab values are
     * [L]ightness : range 0-100
     * [A] (green <-> red): range usually clamped to +- 128
     * [B] (blue <-> yellow): range usually clamped to +- 128
     */
    private double[] setpointLab = {0, 135, 172};
    private Scalar setpointLabScalar = new Scalar(setpointLab);

    private final Scalar lineColor = new Scalar(127, 255, 127);

    /**
     * Difference is expressed as distance within a 3D colorspace
     * Horizontal axes: A/B
     * Vertical axis: L
     *
     * The practical bounds of this threshold are 0 to 375
     * 0 being royal blue and 375 being firetruck red, the two most opposite colors represented
     * by this colorspace
     */
    protected final float labDistanceThresholdSquared = 300;

    private double averageXPosition;
    private double averageYPosition;
    private double m00;

    private Rect largestRect = new Rect();

    private boolean isPipelineReady = false;
    private boolean rectProc = false; // Use rectangle based processing

    private VideoWriter video = new VideoWriter();

    public HueTrackingPipeline() {
        this.setpointLab = new double[]{0, 135, 172};
        setpointLabScalar = new Scalar(this.setpointLab);
    }

    public HueTrackingPipeline(double[] setpointLab) {
        this.setpointLab = setpointLab;
        setpointLabScalar = new Scalar(this.setpointLab);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat originalImage = new Mat();
        originalImage = input.clone();

        int rows = input.rows();
        int cols = input.cols(); // Cache variables

        Mat whiteBalanced = WhiteBalance.whiteBalance(input);
        input.release();

        /**
         *         WARNING: OpenCV is cancerous and converts 8-bit CIELAB values like this:
         *         L <- L*255/100, a <- a + 128, b <- b + 128
         *         This will make the OpenCV Limits of all 3 values 0 <-> 255
         */
        Mat CIELab = new Mat();
        Imgproc.cvtColor(whiteBalanced, CIELab, Imgproc.COLOR_RGB2Lab); // Color Isolation
        //whiteBalanced.release();

        CIELab.convertTo(CIELab, CvType.CV_32F); // Floatify for upcoming calculations
        centerColorLab = CIELab.get((int)rows/2, (int)cols/2); // Get center color

        Mat subtracted = new Mat();
        Core.subtract(CIELab, setpointLabScalar, subtracted); // filtered - setpointLab
        CIELab.release();

        Mat squared = new Mat();
        Core.multiply(subtracted, subtracted, squared); // filtered^2
        subtracted.release();

        List<Mat> LABChannels = new ArrayList<Mat>(); // Split image into L, a, and b channels
        Core.split(squared, LABChannels);
        squared.release();

        LABChannels.set(0, Mat.zeros(rows, cols, LABChannels.get(0).type())); // Set L channel to 0
        Mat merged = new Mat();
        Core.merge(LABChannels, merged); // Merge the channels

        LABChannels.get(0).release();
        LABChannels.get(1).release();
        LABChannels.get(2).release();

        // Add all channels
        Mat reduced = new Mat();
        Core.reduce(merged.reshape(1, cols * rows), reduced, 1, Core.REDUCE_SUM);
        merged.release();

        Mat reshaped = reduced.reshape(1, rows); // This causes a memory leak but it doesn't cause any issues so eh
        reduced.release();

        Mat range = new Mat();
        Core.inRange(reshaped, new Scalar(0), new Scalar(labDistanceThresholdSquared), range); // Check distance

        range.convertTo(range, CvType.CV_8UC3); // Convert to byte before finding center for efficiency

        Mat denoise1 = new Mat(); // Basic deniosing
        Imgproc.morphologyEx(range, denoise1, Imgproc.MORPH_OPEN, new Mat());
        range.release();
        Mat denoise2 = new Mat();
        Imgproc.morphologyEx(denoise1, denoise2, Imgproc.MORPH_CLOSE, new Mat());
        denoise1.release();

        Moments m = Imgproc.moments(denoise2, true); // Caluclate the moments of the image

        double m10 = m.m10;
        double m01 = m.m01;
        m00 = m.m00;

        double averageXPixelPosition = (m10 / m00); // Extract X and Y from moments
        double averageYPixelPosition = (m01 / m00);
        averageXPosition = averageXPixelPosition / ((double) (cols)); // Map to UV space
        averageYPosition = averageYPixelPosition / ((double) (rows));

        if(rectProc) {
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>(); // Find image contours
            Imgproc.findContours(denoise2, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect maxRect = new Rect();

            for (MatOfPoint contour : contours) { // Sort for largest bounding box
                Rect testRect = Imgproc.boundingRect(contour);
                if (testRect.area() > maxArea) {
                    maxRect = testRect;
                    maxArea = testRect.area();
                }
            }

            largestRect = maxRect;
        }

        denoise2.release();

        if (renderLines) { // Render overlays
            if (rectProc) {
                Imgproc.rectangle(whiteBalanced, largestRect, lineColor);
            } else {
                Imgproc.circle(whiteBalanced, new Point(averageXPixelPosition, averageYPixelPosition), 7, lineColor, 2);
            }
        }

        isPipelineReady = true;

        if(video.isOpened()) {
            video.write(whiteBalanced); // Save video frame
        }

        return whiteBalanced;
    }

    public Double getAverageXPosition() {
        return averageXPosition;
    }

    public Double getAverageYPosition() {
        return averageYPosition;
    }

    public Rect getLargestRect() {
        return largestRect;
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
        this.setpointLabScalar = new Scalar(setpointLab);
    }

    public double[] getSetpointLab() {
        return this.setpointLab;
    }

    public double getPixelCount() {
        return m00;
    }

    public void setRectProc(boolean rect) {
        rectProc = rect;
    }

    public void startVideo() {
        int counter = 0;

        File f = new File("/storage/emulated/0/FIRST/OpenCV.avi");

        if(f.exists()) {
            f.renameTo(new File("/storage/emulated/0/FIRST/OpenCV.old.avi"));
        }

        int fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
        video.open("/storage/emulated/0/FIRST/OpenCV.avi", fourcc, 30f, new Size(160, 120));

        renderLines = true;
    }

    public void stopVideo() {
        video.release();

        renderLines = false;
    }
}
