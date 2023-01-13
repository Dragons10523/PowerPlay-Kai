package org.firstinspires.ftc.teamcode.processors;

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class SignalOpticalSystem extends OpenCvPipeline {
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    public enum SignalOrientation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final Scalar[] COLOR_CHECKS = {
            new Scalar(232, 80, 114),  // CYAN
            new Scalar(248, 106, 222), // YELLOW
            new Scalar(154, 226, 67),  // MAGENTA
    };

    public int passes = 0;
    private boolean takePicture = false;
    private String picturePath = "";
    private final CascadeClassifier cascadeClassifier;
    private SignalOrientation signalOrientation = SignalOrientation.RIGHT;
    
    public double[] centerHSV = new double[3];

    public SignalOpticalSystem() {
        cascadeClassifier = new CascadeClassifier("/storage/emulated/0/FIRST/signalClassifiers/wings.xml");
    }

    public SignalOrientation getSignalOrientation() {
        return signalOrientation;
    }

    public boolean isReady() {
        return passes > 0;
    }

    @SuppressLint("DefaultLocale")
    public void takePicture() {
        int imageNumber = 0;

        File f = null;
        do {
            f = new File(String.format("/storage/emulated/0/FIRST/signalImages/signalImage%3d.jpg", imageNumber));
            imageNumber++;
        } while (f.exists());

        picturePath = f.getPath();

        takePicture = true;
    }

    @Override
    public Mat processFrame(Mat input) {
        int maxScore = 0;
        int maxScoreIndex = 1;

        MatOfRect detectedInstances = new MatOfRect();
        cascadeClassifier.detectMultiScale(input, detectedInstances);

        // Get the largest most centered rectangle
        Rect centeredRectangle = new Rect(0, 0, 0, 0);
        double rectangleScore = Double.POSITIVE_INFINITY;
        Rect[] instancesArray = detectedInstances.toArray();
        for(Rect instance : instancesArray) {
            float centerX = instance.x + (instance.width/2f);
            float centerY = instance.y + (instance.height/2f);

            double score = Control.squaredHypotenuse(centerX - (CAMERA_WIDTH/2), centerY - (CAMERA_HEIGHT/2));
            double area = instance.area();

            if(score > rectangleScore) {
                continue;
            } else if(score == rectangleScore && area < centeredRectangle.area()) {
                continue;
            }

            rectangleScore = score;
            centeredRectangle = instance;
        }

        if(centeredRectangle.area() == 0) {
            if(!isReady()) return input; // Return early if nothing was detected from last time

            centeredRectangle = new Rect(135, 75, 90, 90); // Default value if noting is detected
        }

        Mat subMat = input.submat(centeredRectangle); // Image cropping
        Imgproc.rectangle(input, centeredRectangle, new Scalar(64, 255, 64));

        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2Lab);

        centerHSV = hsv.get(CAMERA_HEIGHT/2, CAMERA_WIDTH/2);
        
        /*for(int i = 0; i < COLOR_CHECKS.length; i++) {
            Scalar colorCheck = COLOR_CHECKS[i];

            Mat filteredMat = getColorFilter(hsv, colorCheck);
            int score = Core.countNonZero(filteredMat); // Count the number of matched pixels in the image
            filteredMat.release();

            if(score > maxScore) {
                maxScore = score;
                maxScoreIndex = i;
            }
        }*/
        //hsv.release();

        switch (maxScoreIndex) {
            case 0:
                signalOrientation = SignalOrientation.LEFT;
                break;
            case 2:
                signalOrientation = SignalOrientation.RIGHT;
                break;
            case 1:
            default:
                signalOrientation = SignalOrientation.MIDDLE;
        }

        if(takePicture) {
            takePicture = false;

            Imgcodecs.imwrite(picturePath, input);
        }

        passes++;

        Imgproc.circle(input, new Point(CAMERA_HEIGHT/2, CAMERA_WIDTH/2), 3, new Scalar(0, 255, 0));

        return getColorFilter(hsv, COLOR_CHECKS[0]);
    }

    private Mat getColorFilter(Mat image, Scalar color) {
        image.convertTo(image, CvType.CV_32F); // Floatify for upcoming calculations

        Mat subtracted = new Mat();
        Core.subtract(image, new Scalar(0), subtracted);

        Mat squared = new Mat();
        Core.multiply(subtracted, subtracted, squared);
        subtracted.release();

        Mat reduced = new Mat();
        Core.reduce(squared.reshape(1, image.rows() * image.cols()), reduced, 1, Core.REDUCE_SUM);
        squared.release();

        Mat rangeCheck = new Mat();
        Core.subtract(reduced, new Scalar(900), rangeCheck);

        rangeCheck.convertTo(rangeCheck, CvType.CV_8U);

        return rangeCheck;
    }
}
