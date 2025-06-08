package org.firstinspires.ftc.teamcode.opmodes;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
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


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LEDTesting extends LinearOpMode {
    private String alliance = "BOTH";

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private boolean reverse = true;
    public double power_multiplier = 1.0;

    private DcMotorEx lift;
    private DcMotorEx gantry;

    private Servo wrist;
    private  Servo gripper;

    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Pausing to allow OTOS to initialise");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("OTOS should be initialised");
        telemetry.update();
        telemetry.clear();

        Gamepad.LedEffect rgbEffect1 = new Gamepad.LedEffect.Builder()
                .addStep(0, 0, 1, 2500)
                .build();
        Gamepad.LedEffect rgbEffect2 = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 2500)
                .build();

        String team = "neutral";
        telemetry.addLine("hat team are we on? X for Blue Alliance. CIRCLE for Red Alliance");
        telemetry.update();

        while (team == "neutral")
            if (gamepad1.cross) {
                team = "blue";
                gamepad1.runLedEffect(rgbEffect1);
            }
            else if (gamepad1.circle) {
                team = "red";
                gamepad1.runLedEffect(rgbEffect2);
            }


        while(!isStarted() && !isStopRequested())
        {
            telemetry.update();
            sleep(50);
        }

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {
            ;
        }
    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        Scalar green = new Scalar(255,255,0);
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
        List<MatOfPoint> redContours = new ArrayList<>();
        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();

        MatOfPoint2f largestContour = new MatOfPoint2f();

        Mat points = new Mat();

        Mat lineParams = new MatOfPoint2f();
        float[] line = new float[4];
        Point[] vertices = new Point[4];


        private double sampleAngle = 0.0;


        public Mat processFrame(Mat input)
        {
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
            redContours.clear();
            blueContours.clear();
            contours.clear();

            //find the contours within the yellow, red and blue masks
            Imgproc.findContours(maskYellow, yellowContours, yellowHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(maskRed, redContours, redHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(maskBlue, blueContours, blueHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            //generally speaking we are always interested in yellow samples
            contours.addAll(yellowContours);

            //ignore blue contours unless we are on the blue alliance or testing
            if (alliance == "BLUE" || alliance == "BOTH") {
                contours.addAll(blueContours);
            }
            //ignore red contours unless we are on the red alliance or testing
            if (alliance == "RED" || alliance == "BOTH") {
                contours.addAll(redContours);
            }

            //first safety net - did we detect any contours?
            if (contours.size() > 0) {

                //find the largest contour by area
                double maxVal = 0;
                int maxValIdx = -1;

                for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                    if (maxVal < contourArea) {
                        maxVal = contourArea;
                        maxValIdx = contourIdx;
                    }
                }

                telemetry.addData("largest contour area (pixels)", maxVal);

                //only continue processing if we found a contour and it's big enough to possibly be a block
                if (maxValIdx >= 0 && maxVal > 1000) {

                    //convert largestContour to a MatofPoint2F
                    largestContour = new MatOfPoint2f(contours.get(maxValIdx).toArray());

                    //find the vertices of the smallest possible bounding box (because samples are rectangular)
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

                    telemetry.addData("Angle with x-axis: ", 180-rectangle.angle);
                    this.sampleAngle = 180-rectangle.angle;
                    Moments moments= Imgproc.moments(largestContour);
                    int centrex = (int)(moments.get_m10()/moments.get_m00());
                    int centrey = (int)(moments.get_m01()/moments.get_m00());

                    Imgproc.circle(input,new Point(centrex,centrey),7,new Scalar(255,0,255),-1);
                }
            }

            //always return a frame even if you found no samples
            return input;
        }

        private double dot(double[] a, double[] b) {
            return a[0] * b[0] + a[1] * b[1];
        }
        @Override
        public void onViewportTapped()
        {
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

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }

        public double getSampleAngle() {
            return sampleAngle;
        }

        public void setSampleAngle(double sampleAngle) {
            this.sampleAngle = sampleAngle;
        }
    }
}


