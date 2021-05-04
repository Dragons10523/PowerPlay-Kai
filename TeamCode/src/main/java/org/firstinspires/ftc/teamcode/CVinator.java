package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class CVinator extends OpenCvPipeline {

    List<Mat> channels = new ArrayList<>();
    Mat sub  = new Mat();
    Mat yuv  = new Mat();
    Mat hsv  = new Mat();
    Mat bit  = new Mat();
    Mat blue = new Mat();
    double m10;
    double m00;
    double cols;
    boolean quality = false;

    @Override
    public Mat processFrame(Mat in) {
        sub = in.submat(190,310,0,480);
        Imgproc.cvtColor(sub, yuv, Imgproc.COLOR_RGB2YUV);
        Core.split(yuv,channels);
        Imgproc.equalizeHist(channels.get(0),channels.get(0));
        Core.merge(channels,yuv);
        Imgproc.cvtColor(yuv, sub, Imgproc.COLOR_YUV2RGB);
        Imgproc.cvtColor(sub, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(95,100,50), new Scalar(135,255,255), bit);
        Core.multiply(bit, new Scalar(255), blue);
        return blue;
    }

    public Double averagePosition(){
        Moments m = Imgproc.moments(blue);
        m10 = m.m10;
        m00 = m.m00;
        cols = blue.cols();

        quality = m00 > 4000;

        return (m.m10/m.m00)/((double)(blue.cols()));
    }

}
