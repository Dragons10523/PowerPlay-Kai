package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorEnum;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class ColorPipelineRed extends OpenCvPipeline {

    //HSV_FULL colorspace
    Scalar lowerRed = new Scalar(0, 10, 10);
    Scalar upperRed = new Scalar(20, 255, 255);

    Scalar lowerBlue = new Scalar(135, 75, 80);
    Scalar upperBlue = new Scalar(185, 255, 255);
    Telemetry telemetry;

    int loops = 0;

    double spikeBoundaries;

    public ColorPipelineRed(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public ColorPipelineRed() {
    }

    public enum PieceLocation {
        LEFT,
        CENTER,
        RIGHT,
    }


    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();

        Core.inRange(hsv, lowerRed, upperRed, mask);
        //Core.inRange(hsv, lowerBlue, upperBlue, mask);

        hsv.release();

        Mat hierarchy = new Mat();


        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        hierarchy.release();

        double largestRectArea = 0;
        double largestRectWidth = 0;
        double largestRectHeight = 0;
        Rect largestRectA = null;
        Rect largestRectW = null;
        Rect largestRectH = null;

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);

            if (rect.area() > largestRectArea) {
                largestRectA = rect;
                largestRectArea = rect.area();

            }
            if (rect.width > largestRectWidth) {
                largestRectW = rect;
                largestRectWidth = rect.width;

            }
            if(rect.height > largestRectHeight){
                largestRectH = rect;
                largestRectHeight = rect.height;
            }
            Imgproc.drawContours(input, contours, -1, new Scalar(30, 127, 255));
        }


        if (largestRectW == null || largestRectA == null) {
            return input;
        }


        int centerX = largestRectA.x + largestRectA.width / 2;
        int centerY = largestRectA.y + largestRectA.height / 2;
        Point leftLineStart = new Point(largestRectW.x, input.height());
        Point leftLineStop = new Point(largestRectW.x, 0);
        Point rightLineStart = new Point(largestRectW.x + largestRectWidth, input.height());
        Point rightLineStop = new Point(largestRectW.x + largestRectWidth, 0);
        Imgproc.line(mask, leftLineStart, leftLineStop, new Scalar(125, 255, 255), 4);
        Imgproc.line(mask, rightLineStart, rightLineStop, new Scalar(125, 255, 255), 4);
        int frameCenterX = input.width() / 2;
        if (largestRectArea > 50000) {

            if (centerX < largestRectW.x) {
                ColorPipelineBlue.location = ColorPipelineBlue.PieceLocation.LEFT;
            } else if (centerX > largestRectW.x && centerX < largestRectW.x + largestRectWidth) {
                ColorPipelineBlue.location = ColorPipelineBlue.PieceLocation.CENTER;
            } else if ( centerX > largestRectW.x + largestRectWidth){
                ColorPipelineBlue.location = ColorPipelineBlue.PieceLocation.RIGHT;
            }
        }
        else {
            ColorPipelineBlue.location = ColorPipelineBlue.PieceLocation.RIGHT;
        }


        telemetry.addData("PieceLocation", ColorPipelineBlue.location);
        telemetry.addData("ColorEnum", ColorEnum.color);
        telemetry.addData("largetRectArea", largestRectArea);
        telemetry.addData("center of Gamepiece", centerX);
        telemetry.addData("largetRectW", largestRectW.x);
        telemetry.update();

        return mask;
    }

    public Mat getMask(Mat input) {
        Mat mask = new Mat();

        if (ColorEnum.color == ColorEnum.Color.BLUE_DOWN || ColorEnum.color == ColorEnum.Color.BLUE_UP) {
            Core.inRange(input, lowerBlue, upperBlue, mask);
        } else {
            Core.inRange(input, lowerRed, upperRed, mask);
        }

        return mask;
    }

    public int confidence() throws InterruptedException {
        loops++;
        return loops;
    }
}
