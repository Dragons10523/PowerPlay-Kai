package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class HueTrackingPipeline extends OpenCvPipeline {

    List<Mat> channels = new ArrayList<>();
    Mat bit = new Mat();
    Mat hsv = new Mat();

    double m10;
    double m00;
    double cols;
    boolean quality = false;

    double averageXPosition;
    double averageXPixelPosition;

    double[] centerColor = {0, 0, 0};

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV); // Light equalization
        Core.split(input, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, input);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV); // Color Isolation
        /*Core.inRange(input, new Scalar(44, 40, 60), new Scalar(80, 100, 100), bit);
        Core.multiply(bit, new Scalar(255), input);

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
        }*/

        centerColor = hsv.get((int)hsv.cols()/2, (int)hsv.rows()/2);

        return input;
    }

    public Double getAverageXPosition() {
        return averageXPosition;
    }

    public double[] getCenterColor() {
        return centerColor;
    }
}
