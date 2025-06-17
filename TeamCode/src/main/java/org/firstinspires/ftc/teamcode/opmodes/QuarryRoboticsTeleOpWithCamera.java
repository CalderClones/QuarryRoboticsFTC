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

import org.firstinspires.ftc.teamcode.SamplePipeline;
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
import java.util.Objects;


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

        SamplePipeline samplePipeline = new SamplePipeline(this, webcam, alliance);
        webcam.setPipeline(samplePipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
            public void onError(int errorCode) {
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
        //stateMachine = new GrabberFiniteStateMachine(this, drive, lift, arm, wrist, gripper);

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

        gamepad1.setLedColor(1, 1, 1, LED_DURATION_CONTINUOUS);
        telemetry.addLine("What team are we on? X for Blue Alliance. CIRCLE for Red Alliance");
        telemetry.addLine("What starting position?. LEFT for left, RIGHT for right");
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                alliance = "Blue";
                gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
                gamepad1.runRumbleEffect(rumbleEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
            }
            if (gamepad1.circle) {
                alliance = "Red";
                gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
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
            telemetry.addData("LiftTarget", lift.get_target_height());
            telemetry.addData("LiftHeight", lift.getCurrentHeight());
            telemetry.update();
            sleep(50);
        }

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            stateMachine.update(previousGamepad2, currentGamepad2);

            if (currentGamepad1.left_bumper || currentGamepad1.left_trigger > 0.5) {
                power_multiplier = 0.25;
            } else if (currentGamepad1.right_bumper || currentGamepad1.right_trigger > 0.5) {
                power_multiplier = 1.0;
            } else {
                power_multiplier = 0.4;
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                reverse = !reverse;
            }

            if (currentGamepad2.a && !previousGamepad2.a) { //CROSS button
                if (Objects.equals(gripper.getPosition(), "Open")) {
                    gripper.close();
                } else {
                    gripper.open();
                }
            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {

            }

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {

            }

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {

            }

            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {

            }

            if (currentGamepad2.left_trigger < 0.1 && currentGamepad2.right_trigger < 0.1) {
                wrist.setTargetAngle(0);
            } else if (currentGamepad2.left_trigger >= 0.1)
                wrist.setTargetAngle(-90 / currentGamepad2.left_trigger);
            else if (currentGamepad2.right_trigger >= 0.1)
                wrist.setTargetAngle(90 / currentGamepad2.right_trigger);

            arm.setTargetAngle((gamepad2.right_stick_y * 45) + 45);


            Rotation2d field_transform = drive.localizer.getPose().heading.inverse();

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                lift.nextPresetPosition();
                telemetry.addData("liftPreset", lift.liftTargetPreset);
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                lift.previousPresetPosition();
                telemetry.addData("liftPreset", lift.liftTargetPreset);
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                lift.set_target_height(0);
            }
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                lift.set_target_height(1000);
            }
            //lift.setPower(-currentGamepad2.left_stick_y);
            //gantry.setPower(currentGamepad2.right_stick_y);

            if (reverse) {
                telemetry.addLine("REVERSE");
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y * power_multiplier,
                        -gamepad1.left_stick_x * power_multiplier);

                drive.setDrivePowers(new PoseVelocity2d(
                        input,
                        -gamepad1.right_stick_x * power_multiplier));

            } else {
                Vector2d input = new Vector2d(
                        gamepad1.left_stick_y * power_multiplier,
                        gamepad1.left_stick_x * power_multiplier);

                drive.setDrivePowers(new PoseVelocity2d(
                        input,
                        gamepad1.right_stick_x * power_multiplier));
            }
            drive.updatePoseEstimate();

            telemetry.addData("LiftTarget", lift.get_target_height());
            telemetry.addData("LiftHeight", lift.getCurrentHeight());


            telemetry.addData("LiftTargetTicks", lift.liftMotor.getTargetPosition());
            telemetry.addData("LiftHeightTicks", lift.liftMotor.getCurrentPosition());

            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Show the elapsed game time and wheel power.

            telemetry.addData("front_left", drive.leftFront.getCurrentPosition());
            telemetry.addData("front_right", drive.rightFront.getCurrentPosition());
            telemetry.addData("rear_left", drive.leftBack.getCurrentPosition());
            telemetry.addData("rear_right", drive.rightBack.getCurrentPosition());
            telemetry.update();
        }
    }
}


