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

public class ColorPipeline extends OpenCvPipeline {
    private final Scalar lowerBound;
    private final Scalar upperBound;
    private Mat colorMat = new Mat();
    private Mat hsvMat = new Mat();
    int imageNum;

    public ColorPipeline(Scalar lower, Scalar upper) {
        // Standard HSV stands for Hue, Saturation, and Value. Commonly, H takes values from 0-360, S takes value from 0-100, and V takes values from 0-100.
        // However, OpenCV takes H from 0-179, S from 0-255, and V from 0-255.
        // lowerBound is always darker/less vibrant than upperBound, meaning that all Standard HSV values in lowerBound should be less than those in upperBound
        lowerBound = lower;
        upperBound = upper;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (hsvMat.empty()) {
            hsvMat = new Mat();
        }
        if (colorMat.empty()) {
            colorMat = new Mat();
        }

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerBound, upperBound, colorMat);

        int width = colorMat.cols();
        int height = colorMat.rows();
        int numOfStreams = 5;
        int[] verticalImageCounts = new int[numOfStreams];
        Mat tempMat;

        for (int i = 0; i < numOfStreams; i++) {
            tempMat = new Mat(colorMat, new Rect(i * width / numOfStreams, 0, width / numOfStreams, height));
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

        Core.bitwise_not(colorMat, colorMat);

        return colorMat;
    }

    public int getImageNum()
    {
        return imageNum;
    }
}
