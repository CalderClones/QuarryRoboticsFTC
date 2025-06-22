package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
    private String alliance = "Both";
    private String start_pos = "left";

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private boolean reverse = true;
    public double power_multiplier = 0.4;
    private OpenCvWebcam webcam;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private  GrabberFiniteStateMachine stateMachine;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();



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

        dash.startCameraStream(webcam, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
        stateMachine = new GrabberFiniteStateMachine(this, samplePipeline, drive, lift, arm, wrist, gripper);

        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(50);

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

        telemetry.addLine("Resetting lift");
        telemetry.update();
        lift.reset();
        telemetry.addLine("Lift has been reset");
        telemetry.update();

        telemetry.addLine("Resetting arm");
        telemetry.update();
        arm.reset();
        telemetry.addLine("Arm has been reset");
        telemetry.update();

        gamepad1.setLedColor(1, 1, 1, LED_DURATION_CONTINUOUS);
        telemetry.addLine("What team are we on? X for Blue Alliance. CIRCLE for Red Alliance");
        telemetry.addLine("What starting position?. LEFT for left, RIGHT for right");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                alliance = "Blue";
                gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
                gamepad1.runRumbleEffect(rumbleEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
                telemetry.update();
            }
            if (gamepad1.circle) {
                alliance = "Red";
                gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
                gamepad1.runRumbleEffect(rumbleEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                start_pos = "Left";
                gamepad1.runRumbleEffect(leftEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
                telemetry.update();
            }
            if (gamepad1.dpad_right) {
                start_pos = "Right";
                gamepad1.runRumbleEffect(rightEffect);
                telemetry.addLine(alliance + " Alliance, " + start_pos + " start.");
                telemetry.update();
            }

            sleep(50);
        }

        gamepad2.setLedColor(1,1,1,LED_DURATION_CONTINUOUS);

        ElapsedTime timer = new ElapsedTime();


        samplePipeline.setAlliance(alliance);
        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (previousGamepad1 != null && previousGamepad2 != null && currentGamepad1 != null && currentGamepad2 != null){
                stateMachine.update(previousGamepad2, gamepad2);
            }


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





            if(stateMachine.getGrabberState() == GrabberFiniteStateMachine.GrabberState.DRIVING)
            {
                arm.moveArm(-gamepad2.right_stick_y);
                lift.moveLift(gamepad1.left_stick_y);

                if (currentGamepad2.left_trigger < 0.1 && currentGamepad2.right_trigger < 0.1) {
                    wrist.setTargetAngle(0);
                } else if (currentGamepad2.left_trigger >= 0.1)
                    wrist.setTargetAngle(-90 * currentGamepad2.left_trigger);
                else if (currentGamepad2.right_trigger >= 0.1)
                    wrist.setTargetAngle(90 * currentGamepad2.right_trigger);

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    lift.nextPresetPosition();
                    telemetry.addData("liftPreset", lift.liftTargetPreset);
                }
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    lift.previousPresetPosition();
                    telemetry.addData("liftPreset", lift.liftTargetPreset);
                }
                if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    //lift.setTargetHeight(0);
                }
                if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    //lift.setTargetHeight(1000);
                }

                if (currentGamepad2.a && !previousGamepad2.a) { //CROSS button
                    if (Objects.equals(gripper.getPosition(), "Open")) {
                        gripper.close();
                    } else {
                        gripper.open();
                    }
                }


                //arm.setTargetAngle(Range.clip(gamepad2.right_stick_y * 90, 0, 90));
            }



            //lift.setPower(-currentGamepad2.left_stick_y);
            //gantry.setPower(currentGamepad2.right_stick_y);

            if (reverse) {
                //telemetry.addLine("REVERSE");
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

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : getRunningActions()) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            setRunningActions(newActions);

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            dash.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }

    public List<Action> getRunningActions() {
        return runningActions;
    }

    public void addRunningAction(Action action){
        runningActions.add(action);
    }

    public void setRunningActions(List<Action> runningActions) {
        this.runningActions = runningActions;
    }
}


