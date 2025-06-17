package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
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

@Autonomous(name = "2025 Nationals Autonomous", group = "Auto")
public class QuarryRoboticsAutonomous extends LinearOpMode {
    private String alliance = "Both";
    private String start_pos = "left";

    private OpenCvWebcam webcam;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private GrabberFiniteStateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();

        double NORTH = Math.toRadians(90);
        double EAST = Math.toRadians(0);
        double SOUTH = Math.toRadians(270);
        double WEST = Math.toRadians(180);
        double BASKET = Math.toRadians(225);

        Pose2d initialPose = new Pose2d(-9, -63, NORTH);
        Pose2d chamber = new Pose2d(-10, -36, NORTH);
        Pose2d chamberScore = new Pose2d(-10, -33, NORTH);
        Pose2d sample1 = new Pose2d(-47, -40, NORTH);
        Pose2d sample2 = new Pose2d(-58, -40, NORTH);
        Pose2d sample3 = new Pose2d(-58, -26, WEST);
        Pose2d basket1 = new Pose2d(-56, -56, BASKET);
        Pose2d basket2 = new Pose2d(-56, -56, BASKET);
        Pose2d basket3 = new Pose2d(-56, -56, BASKET);
        Pose2d park = new Pose2d(-24, -12, EAST);

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

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
        stateMachine = new GrabberFiniteStateMachine(this, samplePipeline, drive, lift, arm, wrist, gripper);

        Action driveToChamber = drive.actionBuilder(initialPose)
                .splineToSplineHeading((chamber), NORTH)
                .build();

        Action driveToChamberScore = drive.actionBuilder(chamber)
                .splineToSplineHeading((chamberScore), NORTH)
                .build();

        Action driveToSample1 = drive.actionBuilder(chamberScore)
                .splineToLinearHeading((sample1), WEST)
                .build();

        Action driveToBasket1 = drive.actionBuilder(sample1)
                .splineToSplineHeading(basket1, SOUTH)
                .build();

        Action driveToSample2 = drive.actionBuilder(basket1)
                .splineToLinearHeading((sample2), NORTH)
                .build();

        Action bailToSample2 = drive.actionBuilder(sample1)
                .splineToLinearHeading((sample2), NORTH)
                .build();

        Action driveToBasket2 = drive.actionBuilder(sample2)
                .splineToSplineHeading(basket2, SOUTH)
                .build();

        Action driveToSample3 = drive.actionBuilder(basket2)
                .splineToLinearHeading((sample3), NORTH)
                .build();

        Action bailToSample3 = drive.actionBuilder(sample2)
                .splineToLinearHeading((sample3), NORTH)
                .build();

        Action driveToBasket3 = drive.actionBuilder(sample3)
                .splineToSplineHeading(basket3, SOUTH)
                .build();

        Action driveToPark = drive.actionBuilder(basket3)
                .splineToLinearHeading((park), EAST)
                .build();

        Action bailToPark = drive.actionBuilder(sample3)
                .splineToLinearHeading((park), EAST)
                .build();

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
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        //Score Specimen
        Actions.runBlocking(
                new SequentialAction(
                        driveToChamber,

                        lift.liftToHighChamber(),
                        driveToChamberScore,
                        lift.liftToHighChamberClipped(),
                        gripper.gripperToOpen(),
                        new ParallelAction(
                                lift.liftToScanning(),
                                wrist.wristToHome()
                        ),
                        new ParallelAction(
                                driveToSample1,
                                arm.armToHorizontal()
                        )
                )
        );

        //Handle Sample 1
        timer.reset();
        samplePipeline.startScanning();

        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {
            telemetry.addLine("Scanning for sample");
            telemetry.update();
            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample detected - plotting a course");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            //build a vector allowing the centre of the gripper to move to the centre of the detected sample
            Vector2d sampleLocation = drive.localizer.getPose().position.plus(samplePipeline.sampleLocation);

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(sampleLocation)
                    .build();

            //grab it, score it, go to sample 2
            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(samplePipeline.getSampleAngle())
                            ),
                            gripper.gripperToClosed(),
                            new ParallelAction(
                                    arm.armToVertical(),
                                    driveToBasket1
                            ),
                            lift.liftToHighBasket(),
                            arm.armToBasket(),
                            gripper.gripperToOpen(),
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    arm.armToVertical(),
                                    wrist.wristToHome()
                            ),
                            driveToSample2,
                            arm.armToHorizontal()
                    )
            );
        } else {
            //we didn't find a sample. Time to bail and look for the next one.
            telemetry.addLine("ERROR: Sample not found in time. Moving on to sample 2");
            telemetry.update();
            samplePipeline.stopScanning();
            Actions.runBlocking(
                    bailToSample2
            );
        }

        //Handle Sample 2
        timer.reset();
        samplePipeline.startScanning();

        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {
            telemetry.addLine("Scanning for sample");
            telemetry.update();
            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample detected - plotting a course");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            //build a vector allowing the centre of the gripper to move to the centre of the detected sample
            Vector2d sampleLocation = drive.localizer.getPose().position.plus(samplePipeline.sampleLocation);

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(sampleLocation)
                    .build();

            //grab it, score it, go to sample 3
            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(samplePipeline.getSampleAngle())
                            ),
                            gripper.gripperToClosed(),
                            new ParallelAction(
                                    arm.armToVertical(),
                                    driveToBasket2
                            ),
                            lift.liftToHighBasket(),
                            arm.armToBasket(),
                            gripper.gripperToOpen(),
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    arm.armToVertical(),
                                    wrist.wristToHome()
                            ),
                            driveToSample3,
                            arm.armToHorizontal()
                    )
            );
        } else {
            //we didn't find a sample. Time to bail and look for the next one.
            telemetry.addLine("ERROR: Sample not found in time. Moving on to sample 2");
            telemetry.update();
            samplePipeline.stopScanning();
            Actions.runBlocking(
                    bailToSample3
            );
        }


        //Handle Sample 3
        timer.reset();
        samplePipeline.startScanning();

        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {
            telemetry.addLine("Scanning for sample");
            telemetry.update();
            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample detected - plotting a course");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            //build a vector allowing the centre of the gripper to move to the centre of the detected sample
            Vector2d sampleLocation = drive.localizer.getPose().position.plus(samplePipeline.sampleLocation);

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(sampleLocation)
                    .build();

            //grab it, score it, go to park
            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(samplePipeline.getSampleAngle())
                            ),
                            gripper.gripperToClosed(),
                            new ParallelAction(
                                    arm.armToVertical(),
                                    driveToBasket3
                            ),
                            lift.liftToHighBasket(),
                            arm.armToBasket(),
                            gripper.gripperToOpen(),
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    arm.armToVertical(),
                                    wrist.wristToHome()
                            ),
                            driveToPark
                    )
            );
        } else {
            //we didn't find a sample. Time to bail and look for the next one.
            telemetry.addLine("ERROR: Sample not found in time. Moving on to sample 2");
            telemetry.update();
            samplePipeline.stopScanning();
            Actions.runBlocking(
                    bailToPark
            );
        }

        //Parking
        Actions.runBlocking(
                new SequentialAction(
                        lift.liftToLowChamber(),
                        arm.armToBasket()
                )
        );
    }

}

