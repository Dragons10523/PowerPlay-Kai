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

    double m10;
    double m00;
    double cols;
    boolean quality = false;

    double minH = 30;
    double maxH = 80;

    double averageXPosition;
    double averageXPixelPosition;

    double[] centerColor = {0, 0, 0};

    @Override
    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV); // Light equalization
//        Core.split(input, channels);
//        Imgproc.equalizeHist(channels.get(0), channels.get(0));
//        Core.merge(channels, input);
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // Color Isolation

        //Core.inRange(input, new Scalar(minH, (100/100)*255, (100/100)*255), new Scalar(maxH, (100/100)*255, (100/100)*255), new Scalar(50, 50, 50), input);



        // Loop through pixels and evaluate saturation and value
        for(int x = 0; x < input.cols(); x++) {
            for(int y = 0; y < input.rows(); y++) {
                if(evaluateSatVal(input.get(y, x))) {
                    input.put(y, x, 255d,255d,255d);
                } else {
                    input.put(y, x, 0d,0d,0d);
                }
            }
        }

        //Core.

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        centerColor = input.get((int)input.rows()/2, (int)input.cols()/2);
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
}
