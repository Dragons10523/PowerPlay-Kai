package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVinator extends OpenCvPipeline {

    Mat bgr = new Mat();
    Mat blue = new Mat();

    @Override
    public Mat processFrame(Mat in) {
        Imgproc.cvtColor(in, bgr, Imgproc.COLOR_YUV2BGR);
        Core.extractChannel(bgr, blue, 0);
        return blue;
    }

    public Double averagePosition(){
        Moments m = Imgproc.moments(blue);

        return (m.m10/m.m00)/((double)blue.cols());
    }

}
