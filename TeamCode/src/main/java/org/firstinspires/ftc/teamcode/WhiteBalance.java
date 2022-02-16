package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/*
This code is based on https://gist.github.com/tomykaira/94472e9f4921ec2cf582
 */

public class WhiteBalance {
    public static Mat whiteBalance(Mat input) {
        Mat redHist = new Mat();
        Mat greenHist = new Mat();
        Mat blueHist = new Mat();

        List<Mat> channels = new ArrayList<>(); // Split into channels
        Core.split(input, channels);

        Imgproc.calcHist(channels, new MatOfInt(0), new Mat(), redHist, new MatOfInt(256), new MatOfFloat(0, 256)); // Get histograms for each channel
        Imgproc.calcHist(channels, new MatOfInt(1), new Mat(), blueHist, new MatOfInt(256), new MatOfFloat(0, 256));
        Imgproc.calcHist(channels, new MatOfInt(2), new Mat(), greenHist, new MatOfInt(256), new MatOfFloat(0, 256));

        channels.set(0, correctChannel(channels.get(0), redHist)); // Correct each channel
        channels.set(1, correctChannel(channels.get(1), greenHist));
        channels.set(2, correctChannel(channels.get(2), blueHist));

        Mat whiteBalanced = new Mat();
        Core.merge(channels, whiteBalanced); // Merge channels back

        return whiteBalanced;
    }

    private static Mat correctChannel(Mat channel, Mat hist) {
        float[] histData = new float[(int) (hist.total())]; // Convert histogram matrix into an array
        hist.get(0, 0, histData);

        for(int i = 0; i < 254; i++) { // Make historgram values cumulative
            histData[i+1] += histData[i];
        }

        int size = channel.rows() * channel.cols(); // Caching

        int min = 0;
        while(histData[min] < 0.05 * size) { // Find the 5% darkest pixel value
            min++;
        }

        int max = 255;
        while(histData[max] > 0.95 * size) { // Find the 5% brightest pixel value
            max--;
        }

        if(max < 255) { // Adjust max value
            max++;
        }

        // Remap each pixel so that min = 0 and max = 255
        Mat subtracted = new Mat();
        Core.subtract(channel, new Scalar(min), subtracted);
        channel.release();

        Mat scaled = new Mat();
        Core.multiply(subtracted, new Scalar(255f/(max-min)), scaled);
        subtracted.release();

        return scaled;
    }
}
