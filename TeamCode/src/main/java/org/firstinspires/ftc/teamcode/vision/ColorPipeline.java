package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipeline extends OpenCvPipeline {

    Mat blue = new Mat();
    Mat red = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, blue, Imgproc.COLOR_RGB2RGBA);
        return input;
    }
}
