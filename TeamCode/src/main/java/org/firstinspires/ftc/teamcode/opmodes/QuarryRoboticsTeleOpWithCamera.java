package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.GrabberFiniteStateMachine;
import org.firstinspires.ftc.teamcode.Gripper;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrist;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
public class QuarryRoboticsTeleOpWithCamera extends LinearOpMode {
    private String alliance = "neutral";
    private String start_pos = "left";

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private boolean reverse = true;
    public double power_multiplier = 1.0;
    private OpenCvWebcam webcam;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private  GrabberFiniteStateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        SamplePipeline samplePipeline = new SamplePipeline();
        webcam.setPipeline(samplePipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Webcam Fail", "Webcam generated an error");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
        stateMachine = new GrabberFiniteStateMachine(this, drive, lift, arm, wrist, gripper);

        telemetry.addLine("Pausing to allow OTOS to initialise");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("OTOS should be initialised");
        telemetry.update();
        telemetry.clear();

        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 200)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .build();

        Gamepad.RumbleEffect leftEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 250 mSec
                .build();

        Gamepad.RumbleEffect rightEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 300 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 100)  //  Pause for 250 mSec
                .build();


        Gamepad.LedEffect whiteEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 1, 1, LED_DURATION_CONTINUOUS)
                .build();
        Gamepad.LedEffect yellowEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 1, 0, 2500)
                .build();
        Gamepad.LedEffect blueEffect = new Gamepad.LedEffect.Builder()
                .addStep(0, 0, 1, 100)
                .addStep(0, 0, 0, 100)
                .addStep(0, 0, 1, 100)
                .addStep(0, 0, 0, 100)
                .addStep(0, 0, 1, 100)
                .addStep(0, 0, 0, 100)
                .addStep(0, 0, 1, LED_DURATION_CONTINUOUS)
                .build();
        Gamepad.LedEffect redEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .addStep(1, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .addStep(1, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .addStep(1, 0, 0, 0)
                .build();

        gamepad1.setLedColor(1,1,1,LED_DURATION_CONTINUOUS);
        telemetry.addLine("What team are we on? X for Blue Alliance. CIRCLE for Red Alliance");
        telemetry.addLine("What starting position?. LEFT for left, RIGHT for right");
        while(!isStarted() && !isStopRequested())
        {
            if (gamepad1.cross) {
                alliance  = "Blue";
                gamepad1.setLedColor(0,0,1,LED_DURATION_CONTINUOUS);
                gamepad1.runRumbleEffect(rumbleEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
            }
            if (gamepad1.circle) {
                alliance = "Red";
                gamepad1.setLedColor(1,0,0,LED_DURATION_CONTINUOUS);
                gamepad1.runRumbleEffect(rumbleEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
            }
            if (gamepad1.dpad_left) {
                start_pos = "Left";
                gamepad1.runRumbleEffect(leftEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
            }
            if (gamepad1.dpad_right) {
                start_pos = "Right";
                gamepad1.runRumbleEffect(rightEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
            }
            telemetry.update();
            sleep(50);
        }

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

           //wrist.setPosition(1.0-Range.scale(samplePipeline.getSampleAngle(), -90., 90.0, 0.0, 1.0));

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.left_bumper || currentGamepad1.left_trigger > 0.5) {
                power_multiplier = 0.25;
            }
            else if (currentGamepad1.right_bumper || currentGamepad1.right_trigger > 0.5){
                power_multiplier = 1.0;
            }
            else {
                power_multiplier = 0.4;
            }

            if (currentGamepad1.b && !previousGamepad1.b){
                reverse = !reverse;
            }

            if (currentGamepad2.a && !previousGamepad2.a){
                gripper.open();
            }

            if (currentGamepad2.b && !previousGamepad2.b){
                gripper.close();
            }

         /*   if (currentGamepad2.x && !previousGamepad2.x){
                wrist.setPosition(-1.0);
            }

            if (currentGamepad2.y && !previousGamepad2.y){
                wrist.setPosition(1.0);
            }

            if (currentGamepad2.left_trigger < 0.1 && currentGamepad2.right_trigger < 0.1) {
                wrist.setPosition(0.5);
            }
            else if (currentGamepad2.left_trigger >= 0.1)
                wrist.setPosition(0.5+(currentGamepad2.left_trigger) / 2.0);
            else if (currentGamepad2.right_trigger >= 0.1)
                wrist.setPosition(0.5-(currentGamepad2.right_trigger / 2.0));


          */
            Rotation2d field_transform = drive.localizer.getPose().heading.inverse();

            //lift.setPower(-currentGamepad2.left_stick_y);
            //gantry.setPower(currentGamepad2.right_stick_y);

            if(reverse) {
                telemetry.addLine("REVERSE");
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y * power_multiplier,
                        -gamepad1.left_stick_x * power_multiplier);

                drive.setDrivePowers(new PoseVelocity2d(
                        input,
                        -gamepad1.right_stick_x * power_multiplier));

            }
            else {
                Vector2d input = new Vector2d(
                        gamepad1.left_stick_y * power_multiplier,
                        gamepad1.left_stick_x * power_multiplier);

                drive.setDrivePowers(new PoseVelocity2d(
                        input,
                        gamepad1.right_stick_x * power_multiplier));
            }
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Show the elapsed game time and wheel power.

            telemetry.addData("front_left", drive.leftFront.getCurrentPosition());
            telemetry.addData("front_right",  drive.rightFront.getCurrentPosition());
            telemetry.addData("rear_left",  drive.leftBack.getCurrentPosition());
            telemetry.addData("rear_right",  drive.rightBack.getCurrentPosition());
            telemetry.update();
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
        private boolean sampleDetected = false;



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
                    this.sampleDetected = true;
                    Moments moments= Imgproc.moments(largestContour);
                    int centrex = (int)(moments.get_m10()/moments.get_m00());
                    int centrey = (int)(moments.get_m01()/moments.get_m00());

                    Imgproc.circle(input,new Point(centrex,centrey),7,new Scalar(255,0,255),-1);
                }
                else {
                    this.sampleDetected = false;
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

        public boolean getSampleDetected() {
            return sampleDetected;
        }

        public void setSampleDetected(boolean sampleDetected) {
            this.sampleDetected = sampleDetected;
        }
    }
}


