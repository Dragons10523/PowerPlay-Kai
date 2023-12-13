package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorPipeline extends OpenCvPipeline {

    Mat blue = new Mat();
    Mat red = new Mat();

    Mat mask = new Mat();
    Scalar lowerRed = new Scalar(0, 100, 100);
    Scalar upperRed = new Scalar(10, 255, 255);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, blue, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, lowerRed, upperRed, mask);

        Mat hierarachy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);
            int centerX = rect.x = rect.width / 2;
            int centerY = rect.y + rect.height / 2;

            int frameCenterX = processFrame(input).width() / 2;
            if (centerX < frameCenterX - 50) {
                System.out.println("Red pixel on the LEFT at: (" + centerX + ", " + centerY + ")");
            } else if (centerX > frameCenterX + 50) {
                System.out.println("Red pixel on the RIGHT at: (" + centerX + ", " + centerY + ")");
            } else {
                System.out.println("Red pixel in the MIDDLE at: (" + centerX + ", " + centerY + ")");
            }
        }
        
        return input;
        }


    }
}
