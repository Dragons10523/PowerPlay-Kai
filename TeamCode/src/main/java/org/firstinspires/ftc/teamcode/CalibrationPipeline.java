package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CalibrationPipeline extends OpenCvPipeline {
    final double accuracy = 0.01;
    final int colorPrecision = 5;

    Mat lab = new Mat();
    Mat thresh = new Mat();
    Mat output = new Mat();

    double[] centerColor;
    boolean ready = false;

    @Override
    public Mat processFrame(Mat input) {
        output.release();
        thresh.release();
        lab.release();

        output = input.clone();
        Imgproc.cvtColor(output, lab, Imgproc.COLOR_RGB2Lab);
        Core.inRange(lab, new Scalar(95, 140, 165), new Scalar(150, 165, 195), thresh);

        //Core.multiply(gray, new Scalar(255), gray);

        //Imgproc.threshold(gray, thresh, 80, 255, Imgproc.THRESH_BINARY); //Isolate bright colors

        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat()); //Apply image pre-processing
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.GaussianBlur(thresh, thresh, new Size(21.0, 21.0), 0.00);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); //Locate contours
        Imgproc.drawContours(output, contours, 0, new Scalar(0, 255, 0)); //Draw boxes near contours

        centerColor = lab.get((int)lab.rows()/2, (int)lab.cols()/2);

        //output.put((int)output.rows()/2, (int)output.cols()/2, 0, 255, 0, 0);

        input.release();
        ready = true;
        return output;
/*
        double maxArea = 0;
        MatOfPoint2f largestQuad = null;
        for(MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F); //Convert to MatOfPoint2f

            float epsilon = (float)(accuracy * Imgproc.arcLength(contour2f, true));
            Imgproc.approxPolyDP(contour2f, contour2f, epsilon, true); //Simplify contours to approximate the shape more simply


            Point[] contourArray;
            contourArray = contour2f.toArray();
            if(contourArray.length == 4) { //If 4 sided
                Rect rect = Imgproc.boundingRect(contour2f);

                if(rect.area() > maxArea) { //Save the largest quadrilateral
                    maxArea = rect.area();
                    largestQuad = contour2f;
                }
            }
        }

        if(largestQuad == null) return output; //Safety

        return input.mul(largestQuad.inv());*/
    }

    public boolean isReady(){
        return ready;
    }

    public double[] getColor(){
        return centerColor;
    }
}
