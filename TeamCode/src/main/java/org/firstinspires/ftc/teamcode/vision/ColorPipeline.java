package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    //HSV_FULL colorspace
    Scalar lowerRed = new Scalar(0, 100, 100);
    Scalar upperRed = new Scalar(10, 255, 255);

    Scalar lowerBlue = new Scalar(90, 100, 100);
    Scalar upperBlue = new Scalar(130, 255, 255);
    Telemetry telemetry;
    public ColorPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    PieceLocation location = null;

    public enum PieceLocation {
        LEFT,
        CENTER,
        RIGHT,
    }



    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = getMask(hsv);

        Mat hierarchy = new Mat();
        telemetry.update();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        hierarchy.release();

        double rectArea = 0;
        Rect largestRect = null;

        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);

            if(rect.area() > rectArea){
                largestRect = rect;
                rectArea = largestRect.area();
                telemetry.addData("largestRectArea",largestRect.area() );

            }
        }
        if(largestRect == null){
            return mask;
        }
        int centerX = largestRect.x + largestRect.width /2;
        int centerY = largestRect.y + largestRect.height /2 ;

        int frameCenterX = input.width() /2;

        if(centerX < frameCenterX - 400 ){
            location = PieceLocation.RIGHT;// might be right
        }
        else if(centerX > frameCenterX - 400 ){
            location = PieceLocation.LEFT;
        }
        else if(largestRect.area() != 0){
            location = PieceLocation.CENTER;
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(30, 127, 255));

        return mask;
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
