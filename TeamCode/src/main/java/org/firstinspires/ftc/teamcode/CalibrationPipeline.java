package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CalibrationPipeline extends OpenCvPipeline {
    double[] centerColor;
    boolean ready = false;

    @Override
    public Mat processFrame(Mat input) {
        Mat whiteBalanced = WhiteBalance.whiteBalance(input);
        input.release();

        Mat blurred = new Mat();
        Imgproc.GaussianBlur(whiteBalanced, blurred, new Size(9, 9), 0);;
        whiteBalanced.release();

        Mat lab = new Mat();
        Imgproc.cvtColor(blurred, lab, Imgproc.COLOR_RGB2Lab);
        //blurred.release();

        int halfX = lab.cols()/2;
        int halfY = lab.rows()/3;

        centerColor = lab.get(halfY, halfX);

        lab.release();

        Scalar color = new Scalar(255, 255, 255);
        Imgproc.line(blurred, new Point(0, halfY), new Point(halfX*2, halfY), color);
        Imgproc.line(blurred, new Point(halfX, 0), new Point(halfX, halfY*3), color);

        ready = true;

        return blurred;
    }

    public boolean isReady(){
        return ready;
    }

    public double[] getColor(){
        return centerColor;
    }
}
