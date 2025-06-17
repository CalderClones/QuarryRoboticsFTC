package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    };

    private LinearOpMode opMode;
    private MecanumDrive drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private TrajectoryActionBuilder trajectoryToSample;
    private TrajectoryActionBuilder returnTrajectoryFromSample;

    private boolean sampleDetected;
    private double sampleAngle;
    private Vector2d sampleOffset;
    private String sampleColour;
    private GrabberState grabberState;



    public GrabberFiniteStateMachine(LinearOpMode opMode, SamplePipeline samplePipeline, MecanumDrive drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper) {
        this.opMode = opMode;
        this.drive = drive;
        this.lift = lift;
        this.arm = arm;
        this.wrist = wrist;
        this.gripper = gripper;

        this.setSampleDetected(false);
        this.setSampleAngle(0);
        this.setSampleOffset(new Vector2d(-0.5, -3));


        setGrabberState(GrabberState.DRIVING);

    }

    public void update(Gamepad previous, Gamepad current) {
        switch (getGrabberState()) {
            case SCANNING_REQUESTED:
                //scanning has been requested - is the robot ready?
                if (lift.stationary() && arm.stationary() && wrist.stationary() && gripper.stationary()) {
                    opMode.telemetry.addLine("Robot is stationary - switching state to SCANNING");

                    setGrabberState(GrabberState.SCANNING);
                }

                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }

                break;

            case SCANNING:
                //looking for samples - webcam lets us know when we've found one
                if (isSampleDetected()) {
                    opMode.telemetry.addLine("Sample detected");

                    //rumble the gamepad
                    current.rumble(500);

                    if(Objects.equals(sampleColour, "Yellow")) {
                        current.setLedColor(1,1,0,Gamepad.LED_DURATION_CONTINUOUS);
                    }
                    else if(Objects.equals(sampleColour, "Red")) {
                        current.setLedColor(1,0, 0,Gamepad.LED_DURATION_CONTINUOUS);
                    }
                    if(Objects.equals(sampleColour, "Blue")) {
                        current.setLedColor(0,0,1,Gamepad.LED_DURATION_CONTINUOUS);
                    }

                    setGrabberState(GrabberState.SAMPLE_DETECTED);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }

                break;

            case SAMPLE_DETECTED:
                //we've found a sample - does the driver want to pick it up?
                if (current.x && !previous.x) {
                    opMode.telemetry.addLine("Grab pressed - moving to sample");
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                    setGrabberState(GrabberState.MOVING_ROBOT_TO_SAMPLE);

                    Pose2d currentPose = drive.localizer.getPose();
                    Vector2d samplePosition = new Vector2d(currentPose.position.x + getSampleOffset().x, currentPose.position.y + getSampleOffset().y);
                    Pose2d samplePose = new Pose2d(samplePosition, currentPose.heading);
                    Vector2d returnPosition = new Vector2d(currentPose.position.x, currentPose.position.y);

                    trajectoryToSample = drive.actionBuilder(currentPose)
                            .strafeTo(samplePosition);

                    returnTrajectoryFromSample = drive.actionBuilder(samplePose)
                            .strafeTo(returnPosition);

                    Action driveToSample = trajectoryToSample.build();

                    Actions.runBlocking(
                            new ParallelAction(
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(getSampleAngle())
                            )
                    );

                }
                if (current.b && !previous.b) {
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }
                if (!isSampleDetected()) {
                    opMode.telemetry.addLine("Sample Lost - switching state to SCANNING");
                    current.rumble(100);
                    current.setLedColor(0,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                    setGrabberState(GrabberState.SCANNING);
                }
                break;

            case MOVING_ROBOT_TO_SAMPLE:
                //we're moving to the sample - have we reached it? Lets assume we only entered this state if the action completed.
                if (lift.stationary() && wrist.stationary()) {
                    opMode.telemetry.addLine("Lift in position - picking up sample");

                    Actions.runBlocking(
                            gripper.gripperToClosed()
                    );
                    setGrabberState(GrabberState.GRABBING_SAMPLE);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }

                break;

            case GRABBING_SAMPLE:
                //servos are moving to grasp sample - has the gripper finished closing?
                if (gripper.stationary()) {
                    opMode.telemetry.addLine("Gripper closed - returning to original position");

                    Action returnFromSample = returnTrajectoryFromSample.build();
                    Actions.runBlocking(
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
                    setGrabberState(GrabberState.DRIVING);
                }

                break;

            case SAMPLE_COLLECTED:
                //robot is returning to its initial condition - is it their yet?
                if (lift.stationary() && wrist.stationary()) {
                    opMode.telemetry.addLine("Sample collected - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }
                if (current.b && !previous.b) {
                    opMode.telemetry.addLine("Cancel pressed - switching state to DRIVING");
                    setGrabberState(GrabberState.DRIVING);
                }

                break;

            case DRIVING:
                //we're driving around. No automation until scan button is pressed
                if (current.y && !previous.y) {
                    opMode.telemetry.addLine("Scan pressed - moving lift to scanning position");
                    Actions.runBlocking(
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    arm.armToHorizontal(),
                                    wrist.wristToHome(),
                                    gripper.gripperToOpen()
                            )
                    );
                    setGrabberState(GrabberState.SCANNING_REQUESTED);
                }
                break;
        }
    }
}
