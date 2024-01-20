package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.math.MathContext;

public class ContoursPipeline extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {

        Mat img_gray = new Mat();
        Imgproc.cvtColor(input, img_gray, Imgproc.COLOR_BGR2GRAY);

        Mat thresh = new Mat();
        Imgproc.threshold(img_gray, thresh, 100,255, Imgproc.THRESH_BINARY);



        return thresh;
    }


}
