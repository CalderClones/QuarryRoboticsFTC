package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.DataStore;
import org.firstinspires.ftc.teamcode.Gripper;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SamplePipeline;
import org.firstinspires.ftc.teamcode.TelemetryAction;
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
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(50);

        ElapsedTime timer = new ElapsedTime();

        double NORTH = Math.toRadians(90);
        double EAST = Math.toRadians(0);
        double SOUTH = Math.toRadians(270);
        double WEST = Math.toRadians(180);
        double BASKET = Math.toRadians(225);

        Pose2d initialPose = new Pose2d(-9, -63, NORTH);
        Pose2d chamber = new Pose2d(-10, -35.5, NORTH);
        Pose2d chamberReverse = new Pose2d(-10, -39.5, NORTH);
        Pose2d chamberAgain = new Pose2d(-10, -32.5, NORTH);
        Pose2d sample1 = new Pose2d(-59.5, -43, NORTH);
        Pose2d sample2 = new Pose2d(-69.5, -43, NORTH);
        Pose2d sample3 = new Pose2d(-58, -23, WEST);
        Pose2d basket = new Pose2d(-64, -53, BASKET);
        Pose2d basketReverse = new Pose2d(-57, -51, WEST);
        Vector2d basketBackup = new Vector2d(-57, -51);
        Pose2d park = new Pose2d(-24, -6, EAST);

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
        DataStore.lift = lift;
        arm = new Arm(hardwareMap);
        DataStore.arm = arm;
        wrist = new Wrist(hardwareMap);
        DataStore.wrist = wrist;
        gripper = new Gripper(hardwareMap);
        DataStore.gripper = gripper;

        Action driveToChamber = drive.actionBuilder(initialPose)
                .splineToSplineHeading((chamber), NORTH)
                .build();

        Action driveToChamberReverse = drive.actionBuilder(chamber)
                .setReversed(true)
                .splineToSplineHeading((chamberReverse), SOUTH)
                .build();

        Action driveToChamberAgain = drive.actionBuilder(chamberReverse)
                .splineToSplineHeading((chamberAgain), NORTH)
                .build();


        Action driveToSample1 = drive.actionBuilder(chamber)
                .setReversed(true)
                .splineToLinearHeading(chamberReverse, SOUTH)
                .splineToLinearHeading((sample1), WEST)
                .build();

        Action driveToBasket1 = drive.actionBuilder(sample1)
                .setReversed(true)
                .splineToSplineHeading(basket, SOUTH)
                .build();

        Action driveToBasketReverse = drive.actionBuilder(basket)
                .setReversed(true)
                .splineToSplineHeading(basketReverse, EAST)
                .build();

        Action driveToSample2 = drive.actionBuilder(basketReverse)
                .setReversed(true)
                .splineToLinearHeading((sample2), NORTH)
                .build();

        Action bailToSample2 = drive.actionBuilder(sample1)
                .splineToLinearHeading((sample2), NORTH)
                .build();

        Action driveToBasket2 = drive.actionBuilder(sample2)
                .setReversed(true)
                .splineToSplineHeading(basket, SOUTH)
                .build();

        Action driveToSample3 = drive.actionBuilder(basket)
                .setReversed(true)
                .splineToLinearHeading((sample3), NORTH)
                .build();

        Action bailToSample3 = drive.actionBuilder(sample2)
                .splineToLinearHeading((sample3), NORTH)
                .build();

        Action driveToBasket3 = drive.actionBuilder(sample3)
                .setReversed(true)
                .splineToSplineHeading(basket, SOUTH)
                .build();

        Action driveToPark = drive.actionBuilder(basket)
                .setReversed(true)
                .splineToLinearHeading((park), EAST)
                .build();

        Action bailToPark = drive.actionBuilder(sample3)
                .splineToLinearHeading((park), EAST)
                .build();

        /* OTOS appears to be unreliable
        telemetry.addLine("Pausing to allow OTOS to initialise");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("OTOS should be initialised");
        telemetry.update();
        telemetry.clear();
        */
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
        telemetry.addLine("What starting position?. LEFT for left, RIGHT for right (this hasn't been implemented)");
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

        waitForStart();

        telemetry.addLine("Beginning Autonomous movement");
        telemetry.update();

        if (isStopRequested()) return;

        //Score Specimen
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new TelemetryAction(telemetry, "Driving to Chamber and raising lift"),
                                driveToChamber,
                                wrist.wristToHome(),
                                lift.liftToHighChamber()),
                        new TelemetryAction(telemetry, "Clipping Specimen"),
                        lift.liftToHighChamberClipped(),
                        // Commented out as not sure this sleep is needed
                        //new SleepAction(0.5),
                        new TelemetryAction(telemetry, "Releasing Specimen"),
                        gripper.gripperToOpen(),
                        //new SequentialAction(
                        //        driveToChamberReverse,
                        //        lift.liftToLowBasket(),
                        //        driveToChamberAgain,
                        //        lift.liftToHighChamberClipped()
                        //),
                        new ParallelAction(
                                new TelemetryAction(telemetry, "Driving to sample 1 and moving lift"),
                                lift.liftToScanning(),
                                wrist.wristToHome(),
                                driveToSample1,
                                new SequentialAction(
                                        new SleepAction(1),
                                        new TelemetryAction(telemetry, "Moving arm to horizontal for scanning"),
                                        arm.armToHorizontal()
                                )
                        )
                )
        );

        //Handle Sample 1
        timer.reset();
        samplePipeline.startScanning();
        telemetry.addLine("Scanning for sample 1");
        telemetry.update();
        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {

            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample 1 detected - plotting a course to collect");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            Pose2d currentPose = drive.localizer.getPose(); //field centric pose
            Vector2d sampleVector = samplePipeline.sampleLocation; //robot centric sample vector
            Vector2d rotatedSampleVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(sampleVector); //field centric sample vector

            Vector2d fieldSamplePosition = currentPose.position.plus(rotatedSampleVector); //should rotate and add sample position to generate field centric position of the sample

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(fieldSamplePosition)
                    .build();

            //grab it, score it, go to sample 2
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new TelemetryAction(telemetry,"Moving to pick up sample 1"),
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(samplePipeline.getSampleAngle())
                            ),
                            new TelemetryAction(telemetry,"Cosing Gripper"),
                            gripper.gripperToClosed(),
                            new TelemetryAction(telemetry, "Sample Collected"),
                            new ParallelAction(
                                    new TelemetryAction(telemetry, "Driving to basket"),
                                    new SequentialAction(
                                        arm.armToVertical()
                                            //this delay means the arm first goes vertical (for driving) then after a short delay goes to 45 degrees to deposit
                                    ),
                                    driveToBasket1,
                                    lift.liftToHighBasket()
                            ),
                            arm.armToBasket(),
                            new TelemetryAction(telemetry, "Depositing sample 1 in high basket"),
                            gripper.gripperToOpen(),
                            arm.armToVertical(),
                            new ParallelAction(
                                new TelemetryAction(telemetry, "Backing away from basket"),
                                driveToBasketReverse,
                                new SleepAction(1)
                            ),
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    wrist.wristToHome(),
                                    driveToSample2
                            ),
                            new ParallelAction(
                                arm.armToHorizontal(),
                                    new SleepAction(2)
                            )
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
        telemetry.addLine("Scanning for sample 2");
        telemetry.update();
        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {

            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample 2 detected - plotting a course to collect");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            Pose2d currentPose = drive.localizer.getPose(); //field centric pose
            Vector2d sampleVector = samplePipeline.sampleLocation; //robot centric sample vector
            Vector2d rotatedSampleVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(sampleVector); //field centric sample vector

            Vector2d fieldSamplePosition = currentPose.position.plus(rotatedSampleVector); //should rotate and add sample position to generate field centric position of the sample

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(fieldSamplePosition)
                    .build();

            //grab it, score it, go to sample 3
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new TelemetryAction(telemetry,"Moving to pick up sample 3"),
                                    driveToSample,
                                    wrist.wristToAngle(samplePipeline.getSampleAngle()),
                                    new SleepAction(0.5)
                            ),
                            lift.liftToGrabbing(),
                            new TelemetryAction(telemetry,"Cosing Gripper"),
                            gripper.gripperToClosed(),
                            new TelemetryAction(telemetry, "Sample Collected"),
                            arm.armToVertical(),
                            new ParallelAction(
                                    driveToBasket2,
                                    lift.liftToHighBasket(),
                                    new TelemetryAction(telemetry, "Driving to basket")
                            ),

                        arm.armToBasket(),
                        new TelemetryAction(telemetry, "Depositing sample 2 in high basket"),
                        gripper.gripperToOpen(),
                        arm.armToVertical(),
                        new ParallelAction(
                                new TelemetryAction(telemetry, "Backing away from basket"),
                                driveToBasketReverse),
                        new ParallelAction(
                                lift.liftToScanning(),
                                wrist.wristToHome(),
                                driveToSample3
                        ),
                            arm.armToHorizontal()
                    )
            );
        } else {
            //we didn't find a sample. Time to bail and look for the next one.
            telemetry.addLine("ERROR: Sample not found in time. Moving on to sample 3");
            telemetry.update();
            samplePipeline.stopScanning();
            Actions.runBlocking(
                    bailToSample3
            );
        }


        //Handle Sample 3
        timer.reset();
        samplePipeline.startScanning();
        telemetry.addLine("Scanning for sample 3");
        telemetry.update();
        while (!samplePipeline.isSampleDetected() && timer.milliseconds() < 250) {

            sleep(50);
        }

        if (samplePipeline.isSampleDetected()) {
            //we found a sample - lets grab it and take it to the basket!
            telemetry.addLine("Sample 3 detected - plotting a course to collect");
            telemetry.update();
            //the next line will reset sampleDetected to false, but that's ok as we already know we found one
            samplePipeline.stopScanning();

            Pose2d currentPose = drive.localizer.getPose(); //field centric pose
            Vector2d sampleVector = samplePipeline.sampleLocation; //robot centric sample vector
            Vector2d rotatedSampleVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(sampleVector); //field centric sample vector

            Vector2d fieldSamplePosition = currentPose.position.plus(rotatedSampleVector); //should rotate and add sample position to generate field centric position of the sample

            //make an action to strafe to the sample
            Action driveToSample = drive.actionBuilder(drive.localizer.getPose())
                    .strafeTo(fieldSamplePosition)
                    .build();

            //grab it, score it, go to park
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new TelemetryAction(telemetry,"Moving to pick up sample 3"),
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(samplePipeline.getSampleAngle())
                            ),
                            new TelemetryAction(telemetry,"Cosing Gripper"),
                            gripper.gripperToClosed(),
                            new TelemetryAction(telemetry, "Sample 3 Collected"),
                            new ParallelAction(
                                    new TelemetryAction(telemetry, "Driving to basket"),
                                    new SequentialAction(
                                            arm.armToVertical(),
                                            //this delay means the arm first goes vertical (for driving) then after a short delay goes to 45 degrees to deposit
                                            new SleepAction(0.5),
                                            arm.armToBasket()
                                    ),
                                    driveToBasket3,
                                    lift.liftToHighBasket()
                            ),
                            new TelemetryAction(telemetry, "Depositing sample 3 in high basket"),
                            gripper.gripperToOpen(),
                            arm.armToVertical(),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    new TelemetryAction(telemetry, "Backing away from basket"),
                                    driveToBasketReverse),

                            new ParallelAction(
                                    lift.liftToScanning(),
                                    wrist.wristToHome(),
                                    driveToPark
                            ),
                            arm.armToHorizontal()
                    )
            );
        } else {
            //we didn't find a sample. Time to bail and look for the next one.
            telemetry.addLine("ERROR: Sample not found in time. Parking");
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

