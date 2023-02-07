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

public class SignalOpticalSystem extends OpenCvPipeline {
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    public enum SignalOrientation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private final Scalar[] COLOR_CHECKS = {
            new Scalar(150, 100, 95),  // CYAN
            new Scalar(160, 90, 150),  // GREEN
            new Scalar(140, 170, 128), // MAGENTA
    };

    private final Scalar[] COLOR_OUTLINES = {
            new Scalar(0, 255, 255), // CYAN
            new Scalar(0, 255, 0),   // GREEN
            new Scalar(255, 0, 255), // MAGENTA
    };

    public int passes = 0;
    private boolean takePicture = false;
    private String picturePath = "";
    private final CascadeClassifier cascadeClassifier;
    private SignalOrientation signalOrientation = SignalOrientation.RIGHT;

    private Rect previousRectangle = null;
    
    public double[] centerLab = new double[3];

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
        cascadeClassifier.detectMultiScale(input, detectedInstances, 1.1, 2);

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

        centeredRectangle.height -= 40;
        centeredRectangle.width -= 40;
        centeredRectangle.y += 15;
        centeredRectangle.x += 20;

        if(centeredRectangle.width <= 0 || centeredRectangle.height <= 0) {
            //if(!isReady()) return input; // Return early if nothing was detected from last time

            if(previousRectangle == null)
                centeredRectangle = new Rect(135, 75, 90, 90); // Default value if noting is detected
            else
                centeredRectangle = previousRectangle;
        }

        previousRectangle = centeredRectangle;

        Mat lab = new Mat();
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);

        Mat subMat = lab.submat(centeredRectangle); // Image cropping

        centerLab = lab.get(CAMERA_HEIGHT/2, CAMERA_WIDTH/2);
        
        for(int i = 0; i < COLOR_CHECKS.length; i++) {
            Scalar colorCheck = COLOR_CHECKS[i];

            Mat filteredMat = getColorFilter(subMat, colorCheck);
            int score = Core.countNonZero(filteredMat); // Count the number of matched pixels in the image
            filteredMat.release();

            if(score > maxScore) {
                maxScore = score;
                maxScoreIndex = i;
            }
        }
        lab.release();

        Imgproc.rectangle(input, centeredRectangle, COLOR_OUTLINES[maxScoreIndex]);

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

        return input;
    }

    private Mat getColorFilter(Mat LabImage, Scalar color) {
        LabImage.convertTo(LabImage, CvType.CV_32F); // Floatify for upcoming calculations

        int rows = LabImage.rows();

        Mat subtracted = new Mat();
        Core.subtract(LabImage, color, subtracted);

        Mat squared = new Mat();
        Core.multiply(subtracted, subtracted, squared);
        subtracted.release();

        Mat reducedLightness = new Mat();
        Core.multiply(squared, new Scalar(0.6, 1, 1), reducedLightness);
        squared.release();

        Mat reduced = new Mat();
        Core.reduce(reducedLightness.reshape(1, LabImage.cols() * rows), reduced, 1, Core.REDUCE_SUM);
        reducedLightness.release();

        Mat reshaped = reduced.reshape(1, rows);
        reduced.release();

        Mat inRange = new Mat();
        Core.inRange(reshaped, new Scalar(0), new Scalar(1500), inRange);
        reshaped.release();

        Mat denoise1 = new Mat(); // Basic deniosing
        Imgproc.morphologyEx(inRange, denoise1, Imgproc.MORPH_CLOSE, new Mat());
        inRange.release();
        Mat denoise2 = new Mat();
        Imgproc.morphologyEx(denoise1, denoise2, Imgproc.MORPH_OPEN, new Mat());
        denoise1.release();

        return denoise2;
    }
}
