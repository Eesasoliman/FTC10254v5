package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// Useful Articles:
// https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv
// https://github.com/OpenFTC/EasyOpenCV

public class BlueColorPipeline extends OpenCvPipeline {
    private Mat blueMat = new Mat();
    private Mat hsvMat = new Mat();

    // Standard HSV stands for Hue, Saturation, and Value. Commonly, H takes values from 0-360, S takes value from 0-100, and V takes values from 0-100.
    // However, OpenCV takes H from 0-179, S from 0-255, and V from 0-255.
    // lowerBound is always darker/less vibrant than upperBound, meaning that all Standard HSV values in lowerBound should be less than those in upperBound

    // Old Blue
    private Scalar lowerBound = new Scalar(179 * (200/360d), 255 * (70/100d), 255*(20/100d));
    private Scalar upperBound = new Scalar(179 * (240/360d), 255 * (100/100d), 255*(70/100d));

    int imageNum;
    final int numOfStreams = 5;

    @Override
    public Mat processFrame(Mat input) {
        if (hsvMat.empty()) {
            hsvMat = new Mat();
        }
        if (blueMat.empty()) {
            blueMat = new Mat();
        }

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerBound, upperBound, blueMat);

        int width = blueMat.cols();
        int height = blueMat.rows();
        int[] verticalImageCounts = new int[numOfStreams];
        Mat tempMat;

        for (int i = 0; i < numOfStreams; i++) {
            tempMat = new Mat(blueMat, new Rect(i * width / numOfStreams, 0, width / numOfStreams, height));
            verticalImageCounts[i] = Core.countNonZero(tempMat);
            tempMat.release(); // Release the temporary Mat object
        }

        // Find which of the vertical split images has the most white pixels
        int n = -1;
        for (int i = 0; i < numOfStreams; i++) {
            if (verticalImageCounts[i] > n) {
                n = verticalImageCounts[i];
                imageNum = i + 1;
            }
        }

        return blueMat;
    }

    public int getImageNum()
    {
        return imageNum;
    }
    public int getNumOfStreams() { return numOfStreams; }
    public void setBounds(Scalar lower, Scalar upper)
    {
        lowerBound = lower;
        upperBound = upper;
    }
}
