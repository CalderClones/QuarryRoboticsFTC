package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.QuarryRoboticsTeleOpWithCamera;

import java.util.Objects;

public class GrabberFiniteStateMachine {

    public boolean isSampleDetected() {
        return sampleDetected;
    }

    public void setSampleDetected(boolean sampleDetected) {
        this.sampleDetected = sampleDetected;
    }

    public double getSampleAngle() {
        return sampleAngle;
    }

    public void setSampleAngle(double sampleAngle) {
        this.sampleAngle = sampleAngle;
    }

    public Vector2d getSampleOffset() {
        return sampleOffset;
    }

    public void setSampleOffset(Vector2d sampleOffset) {
        this.sampleOffset = sampleOffset;
    }

    public GrabberState getGrabberState() {
        return grabberState;
    }

    public void setGrabberState(GrabberState grabberState) {
        this.grabberState = grabberState;
    }

    public String getSampleColour() {
        return sampleColour;
    }

    public void setSampleColour(String sampleColour) {
        this.sampleColour = sampleColour;
    }

    public enum GrabberState {
        SCANNING_REQUESTED,
        SCANNING,
        SAMPLE_DETECTED,
        MOVING_ROBOT_TO_SAMPLE,
        GRABBING_SAMPLE,
        SAMPLE_COLLECTED,
        DRIVING

    }

    private final QuarryRoboticsTeleOpWithCamera opMode;
    private final MecanumDrive drive;
    private final Lift lift;
    private final Arm arm;
    private final Wrist wrist;
    private final Gripper gripper;
    private TrajectoryActionBuilder trajectoryToSample;
    private TrajectoryActionBuilder returnTrajectoryFromSample;

    private boolean sampleDetected;
    private double sampleAngle;
    private Vector2d sampleOffset;
    private String sampleColour;
    private GrabberState grabberState;
    private final SamplePipeline samplePipeline;



    public GrabberFiniteStateMachine(QuarryRoboticsTeleOpWithCamera opMode, SamplePipeline samplePipeline, MecanumDrive drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper) {
        this.opMode = opMode;
        this.drive = drive;
        this.lift = lift;
        this.arm = arm;
        this.wrist = wrist;
        this.gripper = gripper;
        this.samplePipeline = samplePipeline;

        this.setSampleDetected(false);
        this.setSampleAngle(0);
        this.setSampleOffset(new Vector2d(3, -0.5));


        setGrabberState(GrabberState.DRIVING);

    }

    public void update(Gamepad previous, Gamepad current) {
        switch (getGrabberState()) {
            case DRIVING:
                //we're driving around. No automation until scan button is pressed
                if (current.y && !previous.y) {
                    opMode.telemetry.addLine("Scan pressed - moving grabber to scanning position");
                    opMode.telemetry.update();
                    opMode.addRunningAction(
                            new SequentialAction(
                                    new ParallelAction(
                                            lift.liftToScanning(),
                                            arm.armToHorizontal(),
                                            wrist.wristToHome()
                                    ),
                                    new ParallelAction(
                                            gripper.gripperToOpen()
                                    )));

                    opMode.telemetry.update();

                    setGrabberState(GrabberState.SCANNING_REQUESTED);
                }
                break;

            case SCANNING_REQUESTED:
                //scanning has been requested - is the robot ready?
                if (opMode.getRunningActions().isEmpty()){//lift.stationary() && arm.stationary() && wrist.stationary() && gripper.stationary()) {
                    opMode.telemetry.addLine("Lift/Arm/Wrist/grabber scanning moves complete - switching state to SCANNING");
                    opMode.telemetry.update();
                    samplePipeline.startScanning();
                    setGrabberState(GrabberState.SCANNING);
                }

                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }

                break;

            case SCANNING:
                //looking for samples - webcam lets us know when we've found one
                if (samplePipeline.isSampleDetected()) {
                    opMode.telemetry.addLine("Sample detected");
                    opMode.telemetry.update();

                    //rumble the gamepad
                    current.rumble(500);

                    if(Objects.equals(samplePipeline.getSampleColour(), "Yellow")) {
                        current.setLedColor(1,1,0,Gamepad.LED_DURATION_CONTINUOUS);
                        opMode.telemetry.addLine("It's Yellow");
                        opMode.telemetry.update();
                }
                    else if(Objects.equals(samplePipeline.getSampleColour(), "Red")) {
                        current.setLedColor(1,0, 0,Gamepad.LED_DURATION_CONTINUOUS);
                        opMode.telemetry.addLine("It's Red");
                        opMode.telemetry.update();
                    }
                    else if(Objects.equals(samplePipeline.getSampleColour(), "Blue")) {
                        current.setLedColor(0,0,1,Gamepad.LED_DURATION_CONTINUOUS);
                        opMode.telemetry.addLine("It's Blue");
                        opMode.telemetry.update();
                    }
                    else{
                        opMode.telemetry.addLine("I don't know what colour it is");
                        opMode.telemetry.update();
                    }

                    setGrabberState(GrabberState.SAMPLE_DETECTED);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }

                break;

            case SAMPLE_DETECTED:
                //we've found a sample - does the driver want to pick it up?
                if (current.x && !previous.x) {
                    opMode.telemetry.addLine("Grab pressed - moving to sample");
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);

                    //Lock the driver out so they don't mess up the grab
                    opMode.lockDriverOut();

                    Pose2d currentPose = drive.localizer.getPose(); //field centric pose
                    Vector2d sampleVector = samplePipeline.sampleLocation; //robot centric sample vector
                    Vector2d rotatedSampleVector = Rotation2d.exp(currentPose.heading.toDouble()-toRadians(90)).times(samplePipeline.sampleLocation); //field centrric sample vector

                    Vector2d fieldSamplePosition = currentPose.position.plus(rotatedSampleVector); //should rotate and add sample position to generate field centric position of the sample

                    Vector2d returnPosition = new Vector2d(currentPose.position.x, currentPose.position.y);

                    opMode.telemetry.addData("Current Position", currentPose.position);
                    opMode.telemetry.addData("Sample Vector", sampleVector);
                    opMode.telemetry.addData("Rotated Sample Vector", rotatedSampleVector);
                    opMode.telemetry.addData("Field Sample Position", fieldSamplePosition);
                    opMode.telemetry.addData("Return Position", returnPosition);

                    trajectoryToSample = drive.actionBuilder(currentPose)
                            .strafeTo(fieldSamplePosition);

                    returnTrajectoryFromSample = drive.actionBuilder(new Pose2d(fieldSamplePosition, currentPose.heading))
                            .strafeTo(returnPosition);

                    Action driveToSample = trajectoryToSample.build();

                    opMode.telemetry.addData("Wrist Target Angle", -samplePipeline.getSampleAngle());

                    opMode.addRunningAction(
                            new SequentialAction(
                                new ParallelAction(
                                        driveToSample,
                                        wrist.wristToAngle(-samplePipeline.getSampleAngle())
                                ),
                                    lift.liftToGrabbing()
                            )
                    );
                    setGrabberState(GrabberState.MOVING_ROBOT_TO_SAMPLE);

                }
                if (current.b && !previous.b) {
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }
                if (!samplePipeline.isSampleDetected()) {
                    opMode.telemetry.addLine("Sample Lost - switching state to SCANNING");
                    current.rumble(100);
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                    setGrabberState(GrabberState.SCANNING);
                }
                break;

            case MOVING_ROBOT_TO_SAMPLE:
                //we're moving to the sample - have we reached it? Lets assume we only entered this state if the action completed.
                if (opMode.getRunningActions().isEmpty()) {
                    opMode.telemetry.addData("Actual Wrist Angle", wrist.getCurrentAngle());
                    opMode.telemetry.addLine("Lift in position - picking up sample");

                    opMode.addRunningAction(
                            gripper.gripperToClosed()
                    );
                    setGrabberState(GrabberState.GRABBING_SAMPLE);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }

                break;

            case GRABBING_SAMPLE:
                //servos are moving to grasp sample - has the gripper finished closing?
                if (opMode.getRunningActions().isEmpty()) {
                    opMode.telemetry.addLine("Gripper closed - returning to original position");

                    Action returnFromSample = returnTrajectoryFromSample.build();
                    opMode.addRunningAction(
                            new ParallelAction(
                                    returnFromSample,
                                    lift.liftToScanning(),
                                    wrist.wristToHome()
                            )
                    );
                    setGrabberState(GrabberState.SAMPLE_COLLECTED);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    opMode.unlockDriver();
                    setGrabberState(GrabberState.DRIVING);
                    samplePipeline.stopScanning();
                }

                break;

            case SAMPLE_COLLECTED:
                //robot is returning to its initial condition - is it their yet?
                if (opMode.getRunningActions().isEmpty()) {
                    opMode.telemetry.addLine("Sample collected - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                    opMode.unlockDriver();
                    samplePipeline.stopScanning();
                }

                break;


        }
    }
}
