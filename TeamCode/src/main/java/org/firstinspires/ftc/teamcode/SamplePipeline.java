package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.opmodes.QuarryRoboticsAutonomous;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class SamplePipeline extends OpenCvPipeline {


    private final OpMode opMode;
    private String alliance;
    private final OpenCvWebcam webcam;
    public Vector2d sampleLocation;
    //TODO: Tune this value
    public Vector2d robotToCamera =  new Vector2d(-9.37 / 25.4, 488.84 / 25.4);
    public Vector2d robotToGripper =  new Vector2d(-14.75 / 25.4, 411.71 / 25.4);
    Vector2d cameraToGripper = robotToCamera.minus(robotToGripper);
    Vector2d cameraToSample = new Vector2d(0,0);
    double PIXELS_PER_INCH = 353 / (350 / 25.4);
            // OLD VERSION PRE NATIONALS: 320.0 / (212 / 25.4); // decrease this value - lets meassure it?
    boolean viewportPaused;
    Scalar green = new Scalar(255, 255, 0);

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    Scalar lowerYellow = new Scalar(18.0, 68.0, 90.0);
    Scalar upperYellow = new Scalar(97.0, 255.0, 255.0);
    Scalar lowerBlue = new Scalar(36.0, 97.0, 30.0);
    Scalar upperBlue = new Scalar(180.0, 252.0, 255.0);
    Scalar lowerRed = new Scalar(0.0, 97.0, 30.0);
    Scalar upperRed = new Scalar(9.0, 252.0, 255.0);
    Mat maskYellow = new Mat();
    Mat maskRed = new Mat();
    Mat maskBlue = new Mat();
    Mat imageHSV = new Mat();
    int kernelSize = 3;
    Mat element = Imgproc.getStructuringElement(0, new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
            new Point(kernelSize, kernelSize));
    Mat blueHierarchy = new Mat();
    Mat yellowHierarchy = new Mat();
    Mat redHierarchy = new Mat();
    List<MatOfPoint> yellowContours = new ArrayList<>();
    List<MatOfPoint> allianceContours = new ArrayList<>();
    List<MatOfPoint> contours = new ArrayList<>();
    MatOfPoint2f largestYellowContour = new MatOfPoint2f();
    MatOfPoint2f largestAllianceContour = new MatOfPoint2f();
    MatOfPoint2f largestContour = new MatOfPoint2f();
    Mat points = new Mat();
    Mat lineParams = new MatOfPoint2f();
    float[] line = new float[4];
    Point[] vertices = new Point[4];
    private boolean scanning;
    private double sampleAngle = 0.0;
    private boolean sampleDetected = false;

    private String sampleColour = "";

    public SamplePipeline(OpMode opMode, OpenCvWebcam webcam, String alliance) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.webcam = webcam;
    }

    public void startScanning() {
        this.scanning = true;
        this.sampleDetected = false;
    }

    public void stopScanning() {
        this.scanning = false;
        this.sampleDetected = false;
    }

    public Mat processFrame(Mat input) {

        //allows auto do disable scanning when not needed to save CPU cycles
        if (scanning) {
            // Convert image to HSV for easier processing
            Imgproc.cvtColor(input, imageHSV, COLOR_RGB2HSV);

            //create masks for pixels that fall into our desired yellow, red and blue rangesx
            inRange(imageHSV, lowerYellow, upperYellow, maskYellow);
            inRange(imageHSV, lowerRed, upperRed, maskRed);
            inRange(imageHSV, lowerBlue, upperBlue, maskBlue);

            //dilate then erode the masks to reduce impact of sensor noise
            Imgproc.dilate(maskYellow, maskYellow, element);
            Imgproc.dilate(maskRed, maskRed, element);
            Imgproc.dilate(maskBlue, maskBlue, element);
            Imgproc.erode(maskYellow, maskYellow, element);
            Imgproc.erode(maskRed, maskRed, element);
            Imgproc.erode(maskBlue, maskBlue, element);

            //clear all the contour lists so they start empty
            yellowContours.clear();
            allianceContours.clear();
            contours.clear();

            //find the contours within the yellow, red and blue masks
            Imgproc.findContours(maskYellow, yellowContours, yellowHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            if(alliance == "Red" || alliance == "Both") {
                Imgproc.findContours(maskRed, allianceContours, redHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }
            if(alliance == "Blue"|| alliance == "Both")
            {
                Imgproc.findContours(maskBlue, allianceContours, blueHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }

            boolean foundContours = false;
            double largestYellowArea = 0;
            double largestAllianceArea = 0;

            // Find largest yellow Contour
            if (!yellowContours.isEmpty()) {

                //find the largest contour by area
                double maxVal = 0;
                int maxValIdx = -1;

                for (int contourIdx = 0; contourIdx < yellowContours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(yellowContours.get(contourIdx));
                    if (maxVal < contourArea) {
                        maxVal = contourArea;
                        maxValIdx = contourIdx;
                    }
                }

                //only continue processing if we found a contour and it's big enough to possibly be a block
                if (maxValIdx >= 0 && maxVal > 1000) {
                    foundContours = true;

                    //convert largestContour to a MatofPoint2F
                    largestYellowContour = new MatOfPoint2f(yellowContours.get(maxValIdx).toArray());
                    largestYellowArea = Imgproc.contourArea(yellowContours.get(maxValIdx));
                }
            }
            else {
                largestYellowContour = null;
                largestYellowArea = 0;
            }

            // Find largest alliance Contour
            if (!allianceContours.isEmpty()) {

                //find the largest contour by area
                double maxVal = 0;
                int maxValIdx = -1;

                for (int contourIdx = 0; contourIdx < allianceContours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(allianceContours.get(contourIdx));
                    if (maxVal < contourArea) {
                        maxVal = contourArea;
                        maxValIdx = contourIdx;
                    }
                }

                //only continue processing if we found a contour and it's big enough to possibly be a block
                if (maxValIdx >= 0 && maxVal > 1000) {
                    foundContours = true;
                    //convert largestContour to a MatofPoint2F
                    largestAllianceContour = new MatOfPoint2f(allianceContours.get(maxValIdx).toArray());
                    largestAllianceArea = Imgproc.contourArea(allianceContours.get(maxValIdx));
                }
            }else
            {
                largestAllianceContour = null;
                largestAllianceArea = 0;
            }



            if (largestYellowArea == 0 && largestAllianceArea == 0) {
                foundContours = false;
            }
            else{
                if (largestYellowArea > largestAllianceArea) {
                    largestContour = largestYellowContour;
                    sampleColour = "Yellow";
                } else {
                    largestContour = largestAllianceContour;
                    sampleColour = alliance;
                }
            }

            if (foundContours)
            {

                //find the vertices of the smallest possible rotated bounding box (because samples are rectangular)
                RotatedRect rectangle = Imgproc.fitEllipse(largestContour);

                Imgproc.minAreaRect(largestContour).points(vertices);
                List<MatOfPoint> boxContours = new ArrayList<>();
                boxContours.add(new MatOfPoint(vertices));


                //draw the bounding box on the input in blue
                Imgproc.drawContours(input, boxContours, 0, new Scalar(0, 0, 255), 5);


                /**
                 * NOTE: to see how to get data from your pipeline to your OpMode as well as how
                 * to change which stage of the pipeline is rendered to the viewport when it is
                 * tapped, please see {@link PipelineStageSwitchingExample}
                 */

                int cols = 240;

                // Prepare the variables


                // Fit line to the largest shape
                //points = new MatOfPoint(vertices);
                Imgproc.fitLine(largestContour, lineParams, Imgproc.DIST_L2, 0, 0.01, 0.01);
                lineParams.get(0, 0, line);

                double vx = line[0];
                double vy = line[1];
                double x = line[2];
                double y = line[3];

                // Calculate left and right of screen y intercepts
                int lefty = (int) ((-x * vy / vx) + y);
                int righty = (int) (((cols - x) * vy / vx) + y);

                // Draw a line across the whole screen to show the major axis of the sample
                Imgproc.line(input, new Point(cols - 1, righty), new Point(0, lefty), new Scalar(0, 255, 0), 2);

                //set up to calculate angle of the line from vertical
                double[] axis = {-1, 0}; // unit vector in the same direction as the zero axis
                double[] yourLine = {vx, vy}; // unit vector in the same direction as your line

                // Calculate the dot product
                double dotProduct = dot(axis, yourLine);
                double angleFromVertical = Math.toDegrees(Math.acos(dotProduct));

                //opMode.telemetry.addData("Angle with x-axis: ", 180 - rectangle.angle);
                this.setSampleAngle(180 - rectangle.angle);
                this.setSampleDetected(true);
                Moments moments = Imgproc.moments(largestContour);
                int centrex = (int) (moments.get_m10() / moments.get_m00());
                int centrey = (int) (moments.get_m01() / moments.get_m00());

                double sampleCentreXInches = (centrex - 120) / PIXELS_PER_INCH;
                double sampleCentreYInches = (160 - centrey) / PIXELS_PER_INCH;

                cameraToSample = new Vector2d(sampleCentreXInches,sampleCentreYInches);
                sampleLocation = cameraToGripper.plus(cameraToSample);

                Imgproc.circle(input, new Point(centrex, centrey), 7, new Scalar(255, 0, 255), -1);
            } else {
                this.setSampleDetected(false);
            }

        } else {
            this.setSampleDetected(false);
        }
        //always return a frame even if you found no samples
        return input;
    }

    private double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    @Override
    public void onViewportTapped() {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        viewportPaused = !viewportPaused;

        if (viewportPaused) {
            webcam.pauseViewport();
        } else {
            webcam.resumeViewport();
        }
    }

    public double getSampleAngle() {
        return sampleAngle;
    }

    public void setSampleAngle(double sampleAngle) {
        this.sampleAngle = sampleAngle;
    }

    public boolean getSampleDetected() {
        return isSampleDetected();
    }

    public void setSampleDetected(boolean sampleDetected) {
        this.sampleDetected = sampleDetected;
    }

    public String getSampleColour() {
        return sampleColour;
    }

    public void setSampleColour(String sampleColour) {
        this.sampleColour = sampleColour;
    }

    public boolean isSampleDetected() {
        return sampleDetected;
    }

    public void setAlliance(String alliance) {
        this.alliance = alliance;
    }
}
