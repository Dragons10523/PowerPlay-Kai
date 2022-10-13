package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

public class SignalOpticalSystem extends OpenCvPipeline {
    public static final int CAMERA_WIDTH = 360;
    public static final int CAMERA_HEIGHT = 240;

    public enum SignalOrientation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final Scalar[] COLOR_CHECKS = {
            new Scalar(180,  255, 255), // CYAN
            new Scalar(300, 255, 255),  // YELLOW
            new Scalar(60,  255, 255)   // MAGENTA
    };

    private boolean isReady = false;
    private boolean takePicture = false;
    private String picturePath = "";
    private CascadeClassifier cascadeClassifier;
    private SignalOrientation signalOrientation = SignalOrientation.RIGHT;

    public SignalOpticalSystem() {
        cascadeClassifier = new CascadeClassifier("/storage/emulated/0/FIRST/imageClassifier.xml");
    }

    public SignalOrientation getSignalOrientation() {
        return signalOrientation;
    }

    public boolean isReady() {
        return isReady;
    }

    @SuppressLint("DefaultLocale")
    public void takePicture() {
        int imageNumber = 0;

        File f = null;
        do {
            f = new File(String.format("/storage/emulated/0/FIRST/signalImage%3d.jpg", imageNumber));
        } while (f.exists());

        picturePath = f.getPath();

        takePicture = true;
    }

    @Override
    public Mat processFrame(Mat input) {
        int maxScore = 0;
        int maxScoreIndex = 2;

        MatOfRect detectedInstances = new MatOfRect();
        cascadeClassifier.detectMultiScale(input, detectedInstances);

        Rect largestRectangle = new Rect(0, 0, 0, 0);
        Rect[] instancesArray = detectedInstances.toArray();
        for(Rect instance : instancesArray) {
            if(instance.area() > largestRectangle.area()) {
                largestRectangle = instance;
            }
        }

        if(largestRectangle.area() == 0) {
            if(isReady) return input; // Return early if nothing was detected from last time

            largestRectangle = new Rect(135, 75, 90, 90); // Default value if noting is detected
        }

        Mat subMat = input.submat(largestRectangle);
        Imgproc.rectangle(input, largestRectangle, new Scalar(64, 255, 64));

        for(int i = 0; i < COLOR_CHECKS.length; i++) {
            Scalar colorCheck = COLOR_CHECKS[i];
            
            Mat hsv = new Mat();
            Imgproc.cvtColor(subMat, hsv, Imgproc.COLOR_RGB2HSV_FULL);

            int score = Core.countNonZero(getColorFilter(hsv, colorCheck)); // Count the number of matched pixels in the image

            if(score > maxScore) {
                maxScore = score;
                maxScoreIndex = i;
            }
        }

        switch (maxScoreIndex) {
            case 0:
                signalOrientation = SignalOrientation.LEFT;
                break;
            case 1:
                signalOrientation = SignalOrientation.MIDDLE;
                break;
            case 2:
            default:
                signalOrientation = SignalOrientation.RIGHT;
        }

        if(takePicture) {
            takePicture = false;

            Imgcodecs.imwrite(picturePath, input);
        }

        isReady = true;

        return input;
    }

    private Mat getColorFilter(Mat image, Scalar color) {
        image.convertTo(image, CvType.CV_32F); // Floatify for upcoming calculations

        Mat subtracted = new Mat();
        Core.subtract(image, color, subtracted);
        image.release();

        Mat squared = new Mat();
        Core.multiply(subtracted, subtracted, squared);
        subtracted.release();

        Mat inRange = new Mat();
        Core.inRange(squared, new Scalar(0, 0, 0), new Scalar(60, 128, 115), inRange);
        squared.release();

        return inRange;
    }
}
