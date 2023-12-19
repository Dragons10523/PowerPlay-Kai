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
    Scalar lowerRed = new Scalar(0, 100, 100);
    Scalar upperRed = new Scalar(10, 255, 255);

    Scalar lowerBlue = new Scalar(90, 100, 100);
    Scalar upperBlue = new Scalar(130, 255, 255);

    PieceLocation location = null;

    public enum PieceLocation {
        LEFT,
        CENTER,
        RIGHT,
    }



    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        Mat mask = getMask(input);

        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double rectArea = 0;
        Rect largestRect = null;

        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);

            if(rect.area() > rectArea){
                largestRect = rect;
                rectArea = rect.area();
            }
        }
        if(largestRect == null){
            return input;
        }
        int centerX = largestRect.x + largestRect.width /2;
        int centerY = largestRect.y + largestRect.height /2 ;

        int frameCenterX = input.width() /2;
        if(centerX < frameCenterX - 200 ){
            location = PieceLocation.RIGHT;// might be right
        }
        else if(centerX > frameCenterX - 200 ){
            location = PieceLocation.LEFT;
        }
        else{
            location = PieceLocation.CENTER;
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0));


        return input;
    }
    public PieceLocation getLocation(){
        return location;
    }
    public Mat getMask(Mat input){
        Mat mask = new Mat();

        if(ColorEnum.color == ColorEnum.Color.BLUE_DOWN || ColorEnum.color == ColorEnum.Color.BLUE_UP){
            Core.inRange(input, lowerBlue, upperBlue, mask);
        }
        else {
            Core.inRange(input, lowerRed, upperRed, mask);
        }

        return mask;
    }
}
