package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import static java.lang.Math.toRadians;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.DataStore;
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
@TeleOp(name = "2025 Nationals TeleOp", group = "drive")
public class QuarryRoboticsTeleOpWithCamera extends LinearOpMode {
    private final String alliance = "Both";
    private final String start_pos = "left";

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();
    private boolean reverse = true;
    public double power_multiplier = 0.4;
    private OpenCvWebcam webcam;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private  GrabberFiniteStateMachine stateMachine;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private boolean driverLockedOut = false;
    private MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(50);


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

        drive = new MecanumDrive(hardwareMap, DataStore.currentPose);
        if(DataStore.lift == null){
            telemetry.addLine("Needed to make a new lift");
            telemetry.update();
            lift = new Lift(hardwareMap);
            telemetry.addLine("Resetting lift");
            telemetry.update();
            lift.reset();
            telemetry.addLine("Lift has been reset");
            telemetry.update();
        }
        else {
            telemetry.addLine("Reusing existing lift");
            telemetry.update();
            lift = DataStore.lift;
            lift.softReset();
        }

        if(DataStore.arm == null){
            telemetry.addLine("Needed to make a new arm");
            telemetry.update();
            arm = new Arm(hardwareMap);
            telemetry.addLine("Resetting arm");
            telemetry.update();
            arm.reset();
            telemetry.addLine("Arm has been reset");
            telemetry.update();
        }
        else{
            telemetry.addLine("Reusing existing arm");
            telemetry.update();
            arm = DataStore.arm;
            arm.softReset();
        }

        if(DataStore.wrist == null){
            telemetry.addLine("Needed to make a new wrist");
            telemetry.update();
            wrist = new Wrist(hardwareMap);
            telemetry.addLine("Better hope it was in the home position :-P");
            telemetry.update();
        }
        else{
            telemetry.addLine("Reusing existing wrist");
            telemetry.update();
            wrist = DataStore.wrist;
            wrist.softReset();
        }

        if(DataStore.gripper == null){
            telemetry.addLine("Needed to make a new gripper");
            telemetry.update();
            gripper = new Gripper(hardwareMap);
            telemetry.addLine("It's a servo, so no resset needed");
            telemetry.update();
        }
        else{
            telemetry.addLine("Reusing existing gripper");
            telemetry.update();
            gripper = DataStore.gripper;
        }

        stateMachine = new GrabberFiniteStateMachine(this, samplePipeline, drive, lift, arm, wrist, gripper);



        /*
        telemetry.addLine("Pausing to allow OTOS to initialise");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("OTOS should be initialised");
        telemetry.update();
        telemetry.clear();
         */

        waitForStart();

        stateMachine.setGrabberState(GrabberFiniteStateMachine.GrabberState.DRIVING);
        gamepad2.setLedColor(1,1,1,LED_DURATION_CONTINUOUS);
        unlockDriver();

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

            //only control availabel to the lift operator when we're not driving is to cancel (to switch state to driving!)
            if(stateMachine.getGrabberState() == GrabberFiniteStateMachine.GrabberState.DRIVING)
            {
                //proportional control for arm
                arm.moveArm(-gamepad2.right_stick_y);
                //proportional control for lift
                lift.moveLift(-gamepad2.left_stick_y);

                //proportional control (albeit weird) for wrist)
                if (currentGamepad2.left_trigger < 0.1 && currentGamepad2.right_trigger < 0.1) {
                    wrist.setTargetAngle(0);
                } else if (currentGamepad2.left_trigger >= 0.1)
                    wrist.setTargetAngle(-90 * currentGamepad2.left_trigger);
                else if (currentGamepad2.right_trigger >= 0.1)
                    wrist.setTargetAngle(90 * currentGamepad2.right_trigger);

                //lift preset handling
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    lift.nextPresetPosition();
                    telemetry.addData("liftPreset", lift.liftTargetPreset);
                }
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    lift.previousPresetPosition();
                    telemetry.addData("liftPreset", lift.liftTargetPreset);
                }

                //TODO: preset handling for arm?
                if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    //lift.setTargetHeight(0);
                }
                if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    //lift.setTargetHeight(1000);
                }

                //gripper control
                if (currentGamepad2.a && !previousGamepad2.a) { //CROSS button
                    if (Objects.equals(gripper.getPosition(), "Open")) {
                        gripper.close();
                    } else {
                        gripper.open();
                    }
                }
            }

            //if the driver isn't locked out, process their stick positions
            if(!driverLockedOut) {
                //Handle crawl/walk.sprint controls
                if (currentGamepad1.left_bumper || currentGamepad1.left_trigger > 0.5) {
                    power_multiplier = 0.25;
                } else if (currentGamepad1.right_bumper || currentGamepad1.right_trigger > 0.5) {
                    power_multiplier = 1.0;
                } else {
                    power_multiplier = 0.4;
                }

                //handle Rey's weird need for reversed controls some times
                if (currentGamepad1.b && !previousGamepad1.b) {
                    reverse = !reverse;
                }

                //use stick controls to tell drivetrain how to move
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
            }
            //if they ARE locked out, ignore all the gamepad controls and stop the drive
            else {

            }

            //always update the pose estimate
            drive.updatePoseEstimate();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : getRunningActions()) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            //ensure that any unexpired actions go back into the queue.
            setRunningActions(newActions);

            //code to draw the robot /camera/gripper/sample
            Pose2d currentPose = drive.localizer.getPose(); //field centric pose
            Vector2d rotatedCameraVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(samplePipeline.robotToCamera); //field centric camera vector
            Vector2d rotatedGripperVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(samplePipeline.robotToGripper); //field centric gripper vector

            Vector2d fieldCameraPosition = currentPose.position.plus(rotatedCameraVector); //should rotate and add sample position to generate field centric position of the camera
            Vector2d fieldGripperPosition = currentPose.position.plus(rotatedGripperVector); //should rotate and add sample position to generate field centric position of the gripper


            if (samplePipeline.isSampleDetected()) {
                Vector2d sampleVector = samplePipeline.sampleLocation; //robot centric sample vector
                Vector2d rotatedSampleVector = Rotation2d.exp(currentPose.heading.toDouble() - toRadians(90)).times(samplePipeline.sampleLocation); //field centric sample vector
                Vector2d fieldSamplePosition = fieldCameraPosition.plus(rotatedSampleVector); //should rotate and add sample position to generate field centric position of the sample
                packet.fieldOverlay().setStroke("#E09F3E");
                Drawing.drawSample(packet.fieldOverlay(), fieldSamplePosition);
            }

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);
            packet.fieldOverlay().setStroke("335C67");
            Drawing.drawCamera(packet.fieldOverlay(), fieldCameraPosition);
            packet.fieldOverlay().setStroke("#9E2A2B");
            Drawing.drawGripper(packet.fieldOverlay(), fieldGripperPosition);

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

    public boolean isDriverLockedOut() {
        return driverLockedOut;
    }

    public void setDriverLockedOut(boolean driverLockedOut) {
        this.driverLockedOut = driverLockedOut;
    }

    public void lockDriverOut() {
        this.driverLockedOut = true;
        telemetry.addLine("DRIVER locked while robot moves");
        gamepad1.setLedColor(1,0,0,LED_DURATION_CONTINUOUS);
        gamepad1.rumble(200);
        Vector2d input = new Vector2d(0,0);
        drive.setDrivePowers(new PoseVelocity2d(
                input,0));
    }
    public void unlockDriver() {
        this.driverLockedOut = false;
        telemetry.addLine("DRIVER is in control");
        gamepad1.setLedColor(0,1,0,LED_DURATION_CONTINUOUS);
        gamepad1.rumble(400);
    }
}


