package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CalibrationPipeline extends OpenCvPipeline {
    Mat lab = new Mat();

    double[] centerColor;
    boolean ready = false;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);

        Imgproc.GaussianBlur(lab, lab, new Size(21, 21), 0.00);
        centerColor = lab.get((int)lab.rows()/2, (int)lab.cols()/2);

        int halfX = lab.cols()/2;
        int halfY = lab.rows()/2;
        Scalar color = new Scalar(255, 255, 255);
        Imgproc.line(lab, new Point(0, halfY), new Point(halfX*2, halfY), color);
        Imgproc.line(lab, new Point(halfX, 0), new Point(halfX, halfY*2), color);

        ready = true;

        return lab;
    }

    public boolean isReady(){
        return ready;
    }

    public double[] getColor(){
        return centerColor;
    }
}
