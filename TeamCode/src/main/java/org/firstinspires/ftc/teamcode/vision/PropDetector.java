package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.PropColor.*;
import static org.firstinspires.ftc.teamcode.vision.PropLocation.*;

import static org.firstinspires.ftc.teamcode.constants.Constants.PropDetectionConstants.*;
import static org.opencv.imgproc.Imgproc.*;

import static org.firstinspires.ftc.teamcode.vision.DisplayView.*;

/**
 * OpenCV pipeline to detect the prop in FTC 2023 - 2024 Centerstage
 */
@Config
public class PropDetector extends OpenCvPipeline {
    public PropColor propColor;

    public static DisplayView VIEW_DISPLAYED = INPUT_WITH_BOUNDING_BOX;

    public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

    private static int erodePasses = ERODE_PASSES;

    PropLocation propLocation = NONE;

    private final Mat hsvMat,
                      lowRedThreshold,
                      highRedThreshold,
                      hierarchy,
                      cvErodeKernel,
                      thresholdOutput,
                      erodeOutput;

    public PropDetector(@NonNull PropColor color) {
        propColor = color;

        hsvMat           = new Mat();
        lowRedThreshold  = new Mat();
        highRedThreshold = new Mat();
        hierarchy        = new Mat();
        cvErodeKernel    = new Mat();
        thresholdOutput  = new Mat();
        erodeOutput      = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, COLOR_RGB2HSV);

        switch (propColor) {
            case RED:
                Core.inRange(
                        hsvMat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, lowRedThreshold);
                Core.inRange(
                        hsvMat, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, highRedThreshold);
                Core.add(lowRedThreshold, highRedThreshold, thresholdOutput);
                break;
            case BLUE:
                Core.inRange(hsvMat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, thresholdOutput);
                break;
        }

        Imgproc.erode(thresholdOutput,
                      erodeOutput,
                      cvErodeKernel,
                      CV_ANCHOR,
                      ERODE_PASSES,
                      CV_BORDER_TYPE,
                      CV_BORDER_VALUE);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(erodeOutput, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        Rect biggestBoundingBox = new Rect(0, 0, 0, 0);

        for (Rect rect : boundRect) {
            if (rect.area() > biggestBoundingBox.area()) {
                biggestBoundingBox = rect;
            }
        }

        if (biggestBoundingBox.area() != 0) {
            propLocation = calculatePropLocation(biggestBoundingBox.x);
        } else {
            propLocation = NONE;
        }

        return hsvMat;
    }

    /**
     * Swaps the prop detector color. Debug function only
     */
    public void swapColor() {
        switch (propColor) {
            case RED:
                propColor = BLUE;
                break;
            case BLUE:
                propColor = RED;
                break;
        }
    }

    /**
     * Increments the amount of times the image is eroded before being passed into the contour
     * detection algorithm.
     */
    public void incrementErodePasses() {
        erodePasses ++;
    }

    /**
     * Decrements the amount of times the image is eroded before being passed into the contour
     * detection algorithm.
     */
    public void decrementErodePasses() {
        erodePasses = Math.max(0, erodePasses--);
    }

    /**
     * Calculates the prop location given the x position of the bounding box.
     * @param xPosition The x position of the largest bounding box present on the screen.
     * @return The location of the prop. Note this function will never return PropLocation.NONE
     */
    private PropLocation calculatePropLocation(int xPosition) {
       if (xPosition < LEFT_X) {
           return LEFT;
       } else if (xPosition > RIGHT_X) {
           return RIGHT;
       }

       return CENTER;
    }

    public PropLocation getPropLocation() { return this.propLocation; }
}

