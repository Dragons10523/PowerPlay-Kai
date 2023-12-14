package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.ColorEnum;
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
    Mat hsv = new Mat();

    Scalar lowerRed = new Scalar(0, 100, 100);
    Scalar upperRed = new Scalar(10, 255, 255);

    Scalar lowerBlue = new Scalar(90, 100, 100);
    Scalar upperBlue = new Scalar(130, 255, 255);

    int location = -1;
    // location 0 is right<------
    // location 1 is center------
    // location 2 is left  ------>





    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        if(ColorEnum.color == ColorEnum.Color.BLUE_DOWN || ColorEnum.color == ColorEnum.Color.BLUE_UP){
            Core.inRange(hsv, lowerBlue, upperBlue, mask);
        }
        else {
            Core.inRange(hsv, lowerRed, upperRed, mask);
        }
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);
            int centerX = rect.x + rect.width /2;
            int centerY = rect.y + rect.height /2 ;

            int frameCenterX = input.width() /2;
            if(centerX < frameCenterX - 200 ){
                location = 0;// might be right
            }
            else if(centerX > frameCenterX - 200 ){
                location = 1;
            }
            else{
                location = 2;
            }
        }

        return mask;
    }
    public int getLocation(){
        return location;
    }

}
